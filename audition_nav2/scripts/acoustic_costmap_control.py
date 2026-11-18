#!/usr/bin/env python3
"""Operator control for the optional Nav2 acoustic soft-cost overlay.

Only invokes the plugin's SetBool/Trigger services; never publishes motion
commands or metric evidence. A disabled/missing plugin fails closed.
"""
import argparse
import sys
import time

import rclpy
from std_srvs.srv import SetBool, Trigger


def invoke(node, service_type, service_name, request, deadline):
    """One bounded RPC, including discovery and response."""
    client = node.create_client(service_type, service_name)
    try:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
            raise RuntimeError("service unavailable: " + service_name)
        future = client.call_async(request)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError("service deadline exceeded: " + service_name)
        rclpy.spin_until_future_complete(node, future, timeout_sec=remaining)
        if not future.done():
            raise RuntimeError("service response timeout: " + service_name)
        response = future.result()
        if response is None:
            raise RuntimeError("empty service response: " + service_name)
        return response
    finally:
        node.destroy_client(client)


def fail_closed_disable(node, endpoint):
    """Best-effort emergency rollback after an unverified enable response.

    Use a fresh short deadline because the original service deadline may have
    expired. A failed rollback must be reported, never treated as success.
    """
    request = SetBool.Request()
    request.data = False
    try:
        result = invoke(node, SetBool, endpoint + "set_soft_costs",
                        request, time.monotonic() + 2.0)
        if not result.success:
            raise RuntimeError(result.message)
        status = invoke(node, Trigger, endpoint + "get_soft_costs_status",
                        Trigger.Request(), time.monotonic() + 2.0)
        fields = dict(token.split("=", 1) for token in status.message.split()
                      if token.count("=") == 1)
        if not status.success or fields.get("enabled") != "false":
            raise RuntimeError("rollback state unverified: " + status.message)
        print("Automatic rollback confirmed: acoustic layer disabled", file=sys.stderr)
        return True
    except Exception as exc:
        print("CRITICAL: automatic rollback could not be confirmed: " + str(exc),
              file=sys.stderr)
        return False


def status_fields(message):
    return dict(token.split("=", 1) for token in message.split()
                if token.count("=") == 1)


def verified_status(node, endpoint, deadline):
    response = invoke(node, Trigger, endpoint + "get_soft_costs_status",
                      Trigger.Request(), deadline)
    if not response.success:
        raise RuntimeError("status rejected: " + endpoint + response.message)
    return status_fields(response.message), response.message


def set_and_verify(node, endpoint, enabled, deadline, revoke=False):
    request = SetBool.Request()
    request.data = enabled
    response = invoke(node, SetBool, endpoint + "set_soft_costs", request, deadline)
    if not response.success:
        raise RuntimeError("set_soft_costs rejected: " + endpoint + response.message)
    fields, message = verified_status(node, endpoint, deadline)
    if fields.get("enabled") != ("true" if enabled else "false"):
        raise RuntimeError("state not confirmed: " + endpoint + message)
    if enabled and (fields.get("terminal_plugin") != "true" or
                    fields.get("trusted_topic_configured") != "true"):
        raise RuntimeError("unsafe enable prerequisites: " + endpoint + message)
    if revoke and fields.get("active_sources") != "0":
        raise RuntimeError("sources not revoked: " + endpoint + message)
    return message


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("status", "enable", "disable", "rollback"))
    parser.epilog = ("Use --costmap both to control local and global overlays together. "
                     "Rollback revokes sources; cost cells clear on the next Nav2 update.")
    parser.add_argument("--costmap", default="/local_costmap/local_costmap",
                        help="Absolute costmap node path, or 'both' for local and global")
    parser.add_argument("--layer", default="acoustic_layer",
                        help="Configured plugin name")
    parser.add_argument("--timeout", type=float, default=3.0,
                        help="Total discovery and response deadline in seconds")
    args = parser.parse_args(argv)
    if not 0 < args.timeout <= 60:
        parser.error("--timeout must be in (0, 60]")
    if not args.layer or "/" in args.layer or not args.layer.replace("_", "a").isalnum():
        parser.error("--layer must be a single ROS name component")
    prefixes = (("/local_costmap/local_costmap", "/global_costmap/global_costmap")
                if args.costmap == "both" else (args.costmap.rstrip("/"),))
    if any(not prefix.startswith("/") or prefix == "/" or "//" in prefix
           for prefix in prefixes):
        parser.error("--costmap must be an absolute costmap node path or 'both'")
    endpoints = [prefix + "/" + args.layer + "/" for prefix in prefixes]

    rclpy.init()
    node = rclpy.create_node("acoustic_costmap_control")
    try:
        deadline = time.monotonic() + args.timeout
        if args.command == "status":
            failed = False
            for endpoint in endpoints:
                try:
                    _, message = verified_status(node, endpoint, deadline)
                    print(endpoint + " " + message)
                except Exception as exc:
                    print("ERROR: " + str(exc), file=sys.stderr)
                    failed = True
            return 1 if failed else 0

        enabling = args.command == "enable"
        failed = False
        if enabling:
            # Do not activate either layer if the other is missing or has
            # invalid admission prerequisites. The actual set remains guarded.
            try:
                for endpoint in endpoints:
                    fields, message = verified_status(node, endpoint, deadline)
                    if (fields.get("terminal_plugin") != "true" or
                            fields.get("trusted_topic_configured") != "true"):
                        raise RuntimeError("unsafe enable prerequisites: " +
                                           endpoint + message)
            except Exception as exc:
                print("ERROR: " + str(exc), file=sys.stderr)
                # Preflight is not a transaction: an earlier operator command
                # may already have enabled one costmap. A failed admission of
                # the pair must not leave that old overlay active while the
                # operator reasonably assumes the group was rejected.
                rollback_ok = True
                for endpoint in endpoints:
                    if not fail_closed_disable(node, endpoint):
                        rollback_ok = False
                if not rollback_ok:
                    print("CRITICAL: preflight rollback unverified; stop Nav2 "
                          "costmap nodes until state is verified", file=sys.stderr)
                    return 3
                return 2

        for endpoint in endpoints:
            try:
                message = set_and_verify(node, endpoint, enabling, deadline,
                                         revoke=args.command == "rollback")
                print(endpoint + " " + message)
            except Exception as exc:
                print("ERROR: " + str(exc), file=sys.stderr)
                failed = True
                if enabling:
                    break
                # Disable/rollback must still try remaining costmaps.

        if failed and args.command in ("enable", "disable", "rollback"):
            # A timed-out SetBool may still complete on the Nav2 executor.
            # In particular, a failed disable/rollback must never be treated
            # as an ordinary CLI failure while an overlay might remain live.
            # Use independent deadlines and attempt every endpoint, including
            # ones whose initial request appeared successful.
            rollback_ok = True
            for endpoint in endpoints:
                if not fail_closed_disable(node, endpoint):
                    rollback_ok = False
            if not rollback_ok:
                print("CRITICAL: one or more acoustic layers may remain enabled; "
                      "stop Nav2 costmap nodes until state is verified",
                      file=sys.stderr)
                return 3
        return 1 if failed else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
