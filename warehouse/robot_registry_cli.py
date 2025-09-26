#!/usr/bin/env python3
"""
Robot Registry CLI

Provides administrative commands for managing robots:
- register: add a robot (with optional name)
- register-bulk: add multiple robots with auto-generated IDs
- delete: remove a robot
- delete-all: remove all robots
- charge-all: set all batteries to 100%
- clear-positions: nullify positions for all robots
- list: list registered robots

Usage examples:
  python -m warehouse.robot_registry_cli register --id R1 --name Alpha
  python -m warehouse.robot_registry_cli register-bulk --count 5
  python -m warehouse.robot_registry_cli delete --id R1
  python -m warehouse.robot_registry_cli delete-all
  python -m warehouse.robot_registry_cli charge-all
  python -m warehouse.robot_registry_cli clear-positions
  python -m warehouse.robot_registry_cli list
"""
import sys
import os
import argparse
import logging

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.impl.robot_registry_impl import RobotRegistryImpl


def main() -> int:
    parser = argparse.ArgumentParser(prog="robot_registry", description="Robot Registry Administration")
    sub = parser.add_subparsers(dest="command", required=True)

    p_register = sub.add_parser("register", help="Register a new robot")
    p_register.add_argument("--id", required=True, help="Robot ID (unique)")
    p_register.add_argument("--name", required=False, help="Human-friendly name")

    p_register_bulk = sub.add_parser("register-bulk", help="Register multiple robots with auto-generated IDs")
    p_register_bulk.add_argument("--count", type=int, required=True, help="Number of robots to register")
    p_register_bulk.add_argument("--name-prefix", default="Robot", help="Prefix for auto-generated names (default: Robot)")

    p_delete = sub.add_parser("delete", help="Delete a robot")
    p_delete.add_argument("--id", required=True, help="Robot ID")

    sub.add_parser("delete-all", help="Delete all robots and their runtime state")
    sub.add_parser("charge-all", help="Charge all robots to 100%")
    sub.add_parser("clear-positions", help="Clear positions for all robots")
    sub.add_parser("list", help="List registered robots")

    args = parser.parse_args()

    # Minimal WarehouseMap to satisfy SimulationDataServiceImpl (layout not used here)
    warehouse_map = WarehouseMap()
    sds = SimulationDataServiceImpl(warehouse_map)
    registry = RobotRegistryImpl(sds)

    try:
        if args.command == "register":
            ok = registry.register_robot(args.id, args.name)
            if ok:
                print(f"[SUCCESS] Registered robot: {args.id} ({args.name or 'auto-name'})")
                return 0
            else:
                print(f"[INFO] Robot already exists: {args.id}")
                return 0
        elif args.command == "register-bulk":
            if args.count <= 0:
                print("[ERROR] Count must be positive")
                return 1
            created = registry.register_robots(args.count, args.name_prefix)
            if created:
                print(f"[SUCCESS] Registered {len(created)} robots:")
                for robot in created:
                    print(f"  - {robot.robot_id} ({robot.name})")
                return 0
            else:
                print("[ERROR] No robots were registered")
                return 1
        elif args.command == "delete":
            ok = registry.delete_robot(args.id)
            if ok:
                print(f"[SUCCESS] Deleted robot: {args.id}")
                return 0
            else:
                print(f"[WARNING] Robot not found: {args.id}")
                return 1
        elif args.command == "delete-all":
            count = registry.delete_all_robots()
            print(f"[SUCCESS] Deleted all robots (removed {count} robots)")
            return 0
        elif args.command == "charge-all":
            affected = registry.charge_all_to_full()
            print(f"[SUCCESS] Charged all robots to full (affected rows: {affected})")
            return 0
        elif args.command == "clear-positions":
            affected = registry.clear_all_positions()
            print(f"[SUCCESS] Cleared positions (affected rows: {affected})")
            return 0
        elif args.command == "list":
            robots = registry.list_registered_robots()
            if robots:
                print(f"[INFO] Found {len(robots)} registered robots:")
                for r in robots:
                    name_part = f" ({r.name})" if r.name else ""
                    print(f"  - {r.robot_id}{name_part}")
            else:
                print("[INFO] No robots registered")
            return 0
        else:
            parser.print_help()
            return 1
    except Exception as e:
        logging.error("Registry CLI failed: %s", e, exc_info=True)
        print(f"[ERROR] {e}")
        return 2


if __name__ == "__main__":
    sys.exit(main())


