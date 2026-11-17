#!/usr/bin/env python3
"""Operator control for the optional Nav2 acoustic soft-cost overlay.

Only invokes the plugin's SetBool/Trigger services; never publishes motion
commands or metric evidence. A disabled/missing plugin fails closed.
"""
import argparse
import sys

import rclpy
from std_srvs.srv import SetBool, Trigger


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("status", "enable", "disable"))
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
        service_type = Trigger if args.command == "status" else SetBool
        service = endpoint + ("get_soft_costs_status" if args.command == "status"
                              else "set_soft_costs")
        client = node.create_client(service_type, service)
        if not client.wait_for_service(timeout_sec=args.timeout):
            print("ERROR: service unavailable: " + service, file=sys.stderr)
            return 2
        request = service_type.Request()
        if args.command != "status":
            request.data = args.command == "enable"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            print("ERROR: service response timeout", file=sys.stderr)
            return 2
        try:
            response = future.result()
        except Exception as exc:
            print("ERROR: service call failed: " + str(exc), file=sys.stderr)
            return 2
        if response is None:
            print("ERROR: empty service response", file=sys.stderr)
            return 2
        print(response.message)
        return 0 if response.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
