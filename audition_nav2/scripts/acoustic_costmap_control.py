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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("status", "enable", "disable", "rollback"))
    parser.epilog = ("rollback disables acoustic admission and verifies source revocation; "
                     "Nav2 clears previously written soft costs on its next map update.")
    parser.add_argument("--costmap", default="/local_costmap/local_costmap",
                        help="Costmap node namespace (default: local Nav2 costmap)")
    parser.add_argument("--layer", default="acoustic_layer",
                        help="Configured plugin name")
    parser.add_argument("--timeout", type=float, default=3.0,
                        help="Maximum service discovery and response wait (seconds)")
    args = parser.parse_args(argv)
    if not 0 < args.timeout <= 60:
        parser.error("--timeout must be in (0, 60]")
    if not args.layer or "/" in args.layer or not args.layer.replace("_", "a").isalnum():
        parser.error("--layer must be a single ROS name component")
    prefix = args.costmap.rstrip("/")
    if not prefix.startswith("/") or prefix == "/":
        parser.error("--costmap must be an absolute costmap node path")
    endpoint = prefix + "/" + args.layer + "/"
    rclpy.init()
    node = rclpy.create_node("acoustic_costmap_control")
    try:
        deadline = time.monotonic() + args.timeout
        status_service = endpoint + "get_soft_costs_status"
        try:
            if args.command == "status":
                response = invoke(node, Trigger, status_service,
                                  Trigger.Request(), deadline)
                print(response.message)
                return 0 if response.success else 1

            control_service = endpoint + "set_soft_costs"
            request = SetBool.Request()
            request.data = args.command == "enable"
            response = invoke(node, SetBool, control_service, request, deadline)
            print(response.message)
            if not response.success:
                return 1

            # A SetBool acknowledgement is not proof that Nav2 retained the
            # requested operator state. Verify the authoritative plugin state
            # before reporting success, within the SAME timeout budget.
            status = invoke(node, Trigger, status_service,
                            Trigger.Request(), deadline)
            if not status.success:
                print("ERROR: plugin status rejected verification", file=sys.stderr)
                return 1
            fields = dict(token.split("=", 1) for token in status.message.split()
                          if token.count("=") == 1)
            expected = "true" if request.data else "false"
            if fields.get("enabled") != expected:
                print("ERROR: plugin state differs from requested state: " +
                      status.message, file=sys.stderr)
                return 1
            if args.command == "rollback" and fields.get("active_sources") != "0":
                print("ERROR: rollback has not revoked all acoustic sources: " +
                      status.message, file=sys.stderr)
                return 1
            if request.data and (fields.get("terminal_plugin") != "true" or
                                 fields.get("trusted_topic_configured") != "true"):
                print("ERROR: enabled plugin failed admission prerequisites: " +
                      status.message, file=sys.stderr)
                return 1
            print(status.message)
            return 0
        except Exception as exc:
            print("ERROR: " + str(exc), file=sys.stderr)
            return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
