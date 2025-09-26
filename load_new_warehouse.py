#!/usr/bin/env python3
"""
Warehouse Data Generation Script - Professional Implementation

REQUIRED ARGUMENTS:
    --warehouse FILE    Warehouse CSV file (required, no default)

Generates and persists:
- Navigation graph (conflict boxes, nodes, edges)
- Shelves from warehouse map
- Inventory with demo data
- All warehouse-related data in database

Design Principles:
- Single Responsibility: Only generates warehouse data
- Dependency Inversion: Uses interfaces for data services
- Error Handling: Comprehensive validation and error reporting
- Logging: Clear progress reporting for all operations
- Validation: Ensures warehouse file exists and is valid

Usage:
    python load_new_warehouse.py --warehouse my_warehouse.csv

Exit Codes:
    0 - Success
    1 - Argument error or file not found
    2 - Warehouse generation failed
    3 - Database error
"""
import sys
import os
import logging
import argparse
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from config.configuration_provider import ConfigurationProvider
from interfaces.graph_persistence_interface import GraphPersistenceResult

# Safe print wrapper to avoid UnicodeEncodeError on Windows consoles
def _safe_print(*args, **kwargs):
    try:
        print(*args, **kwargs)
    except UnicodeEncodeError:
        safe_args = tuple(str(a).encode('ascii', 'ignore').decode() for a in args)
        print(*safe_args, **kwargs)

# Use safe print for all output
def log_info(message: str) -> None:
    _safe_print(f"[INFO] {message}")

def log_error(message: str) -> None:
    _safe_print(f"[ERROR] {message}")

def log_warning(message: str) -> None:
    _safe_print(f"[WARNING] {message}")

def log_success(message: str) -> None:
    _safe_print(f"[SUCCESS] {message}")

def log_process(message: str) -> None:
    _safe_print(f"[PROCESS] {message}")


@dataclass
class WarehouseGenerationResult:
    """Result of warehouse generation operation."""
    navigation_graph: GraphPersistenceResult
    shelves_created: int
    inventory_created: int
    success: bool
    errors: List[str]


class WarehouseGenerator:
    """
    Handles warehouse data generation with clean architecture.

    Responsibilities:
    - Load warehouse map from CSV
    - Generate navigation graph and persist to database
    - Create shelves from warehouse layout
    - Populate inventory with demo data
    """

    def __init__(self, warehouse_csv: str):
        """
        Initialize warehouse generator.

        Args:
            warehouse_csv: Path to warehouse CSV file (required)

        Raises:
            FileNotFoundError: If warehouse CSV doesn't exist
            ValueError: If warehouse CSV is invalid
        """
        self.warehouse_csv = warehouse_csv
        self.warehouse_map: Optional[WarehouseMap] = None
        self.simulation_data_service: Optional[SimulationDataServiceImpl] = None
        self.config_provider: Optional[ConfigurationProvider] = None
        self._csv_hash: Optional[str] = None

    @staticmethod
    def _calculate_file_hash(file_path: str) -> str:
        """Calculate SHA256 hash of a file for change detection."""
        import hashlib
        try:
            with open(file_path, 'rb') as f:
                return hashlib.sha256(f.read()).hexdigest()
        except Exception as e:
            log_warning(f"Could not calculate hash for {file_path}: {e}")
            return ""

    def _validate_warehouse_file(self) -> None:
        """Validate that warehouse CSV file exists and is readable."""
        if not os.path.exists(self.warehouse_csv):
            raise FileNotFoundError(f"Warehouse file not found: {self.warehouse_csv}")

        if not os.path.isfile(self.warehouse_csv):
            raise ValueError(f"Path is not a file: {self.warehouse_csv}")

        if not os.access(self.warehouse_csv, os.R_OK):
            raise PermissionError(f"Cannot read warehouse file: {self.warehouse_csv}")

        log_success(f"Warehouse file validated: {self.warehouse_csv}")

    def _initialize_components(self) -> None:
        """Initialize warehouse map and data services."""
        try:
            # Load warehouse map
            self.warehouse_map = WarehouseMap(csv_file=self.warehouse_csv)
            log_success(f"Warehouse map loaded: {self.warehouse_map.width}x{self.warehouse_map.height}")

            # Calculate CSV hash for later persistence
            self._csv_hash = self._calculate_file_hash(self.warehouse_csv)
            if self._csv_hash:
                log_info(f"Warehouse CSV hash: {self._csv_hash[:8]}...")
            else:
                log_warning("Warehouse CSV hash unavailable; change detection disabled")

            # Initialize configuration provider
            self.config_provider = ConfigurationProvider()
            log_success("Configuration provider initialized")

            # Initialize simulation data service
            self.simulation_data_service = SimulationDataServiceImpl(
                self.warehouse_map, pool_size=5  # Smaller pool for generation
            )
            log_success("Simulation data service initialized")

        except Exception as e:
            log_error(f"Component initialization failed: {e}")
            raise

    def _ensure_sample_robot_fleet(self) -> None:
        """
        Ensure a sample robot fleet exists in the database.

        Checks if any robots are registered, and if none exist,
        registers 3 sample robots to provide a basic fleet for testing.
        This enables immediate simulation without requiring separate
        robot registration steps.

        Design Principles:
        - Single Responsibility: Only ensures sample robots exist
        - Error Handling: Graceful fallback with logging
        - Performance: Minimal database queries
        - User Feedback: Clear logging of actions taken
        """
        try:
            from warehouse.impl.robot_registry_impl import RobotRegistryImpl  # local import to avoid hard dependency if unused

            # Create robot registry instance
            registry = RobotRegistryImpl(self.simulation_data_service)

            # Check existing robots
            existing_robots = registry.list_registered_robots()
            robot_count = len(existing_robots)

            if robot_count > 0:
                log_info(f"Found {robot_count} existing robots - using existing fleet")
                log_info(f"   🤖 Existing robots: {[r.name for r in existing_robots]}")
                return

            # No robots found - create sample fleet
            log_info("No robots found in database - creating sample fleet")

            # Register 3 sample robots
            created_robots = registry.register_robots(count=3, name_prefix="SampleRobot")

            if created_robots:
                robot_names = [r.name for r in created_robots]
                robot_ids = [r.robot_id for r in created_robots]
                log_success(f"Created sample robot fleet: {len(created_robots)} robots")
                log_info(f"   🤖 Sample robots: {robot_names}")
                log_info(f"   🆔 Robot IDs: {[rid[:8] + '...' for rid in robot_ids]}")
            else:
                log_warning("Failed to create sample robot fleet - no robots registered")

        except Exception as e:
            log_warning(f"Failed to ensure sample robot fleet: {e}")
            log_info("   ℹ️  This is non-critical - you can register robots manually later")

    def _generate_navigation_graph(self) -> GraphPersistenceResult:
        """
        Generate and persist navigation graph from warehouse CSV.

        Returns:
            GraphPersistenceResult with persistence statistics
        """
        log_process("Generating navigation graph from warehouse CSV...")

        try:
            result = self.simulation_data_service.persist_navigation_graph_from_csv(
                Path(self.warehouse_csv), clear_existing=True
            )

            log_success(f"Navigation graph generated: {result.boxes_persisted} boxes, "
                       f"{result.nodes_persisted} nodes, {result.edges_persisted} edges")

            return result

        except Exception as e:
            log_error(f"Navigation graph generation failed: {e}")
            raise

    def _create_shelves(self) -> int:
        """
        Create shelves from warehouse map.

        Returns:
            Number of shelves created
        """
        log_process("Creating shelves from warehouse map...")

        try:
            shelves_created = self.simulation_data_service.create_shelves_from_map(clear_existing=True)
            log_success(f"Created {shelves_created} shelves from warehouse map")
            return shelves_created

        except Exception as e:
            log_error(f"Shelves creation failed: {e}")
            raise

    def _populate_inventory(self) -> int:
        """
        Populate shelves with demo inventory.

        Returns:
            Number of inventory items created
        """
        log_process("Populating demo inventory...")

        try:
            # Demo inventory data
            demo_inventory = [
                {'item_id': 'laptop_001', 'name': 'Gaming Laptop', 'quantity': 5},
                {'item_id': 'phone_001', 'name': 'Smartphone', 'quantity': 10},
                {'item_id': 'tablet_001', 'name': 'Tablet', 'quantity': 8},
                {'item_id': 'headphones_001', 'name': 'Wireless Headphones', 'quantity': 15},
                {'item_id': 'keyboard_001', 'name': 'Mechanical Keyboard', 'quantity': 12},
                {'item_id': 'mouse_001', 'name': 'Gaming Mouse', 'quantity': 20},
                {'item_id': 'monitor_001', 'name': '4K Monitor', 'quantity': 6},
                {'item_id': 'speaker_001', 'name': 'Bluetooth Speaker', 'quantity': 9},
                {'item_id': 'camera_001', 'name': 'Action Camera', 'quantity': 7},
                {'item_id': 'drone_001', 'name': 'Quadcopter Drone', 'quantity': 4},
                {'item_id': 'gaming_console_001', 'name': 'Gaming Console', 'quantity': 3},
                {'item_id': 'vr_headset_001', 'name': 'VR Headset', 'quantity': 8},
                {'item_id': 'book_001', 'name': 'Programming Book', 'quantity': 15},
                {'item_id': 'audio_001', 'name': 'Audio Equipment', 'quantity': 6},
            ]

            inventory_created = self.simulation_data_service.populate_inventory(demo_inventory)

            log_success(f"Populated inventory with {inventory_created} items")

            # Export initial inventory status
            if self.simulation_data_service.export_inventory_status_csv("generated_inventory_status.csv"):
                log_info("Initial inventory status exported: generated_inventory_status.csv")
            else:
                log_warning("Failed to export initial inventory status")

            return inventory_created

        except Exception as e:
            log_error(f"Inventory population failed: {e}")
            raise

    def generate_warehouse_data(self) -> WarehouseGenerationResult:
        """
        Generate complete warehouse data set.

        Returns:
            WarehouseGenerationResult with detailed results
        """
        log_info("🚀 Starting warehouse data generation...")
        log_info(f"   📁 Warehouse CSV: {self.warehouse_csv}")

        errors = []

        try:
            # Step 1: Validate warehouse file
            self._validate_warehouse_file()

            # Step 2: Initialize components
            self._initialize_components()

            # Step 3: Generate navigation graph
            navigation_graph = self._generate_navigation_graph()

            # Step 4: Create shelves
            shelves_created = self._create_shelves()

            # Step 5: Populate inventory
            inventory_created = self._populate_inventory()

            # Step 5.1: Check for existing robots and create sample fleet if needed
            self._ensure_sample_robot_fleet()

            # Step 5.2: Clear all robot positions to avoid stale placements in new warehouse
            try:
                from warehouse.impl.robot_registry_impl import RobotRegistryImpl  # local import to avoid hard dependency if unused
                registry = RobotRegistryImpl(self.simulation_data_service)
                affected = registry.clear_all_positions()
                log_info(f"Cleared robot positions after warehouse load (affected rows: {affected})")
            except Exception as e:
                log_warning(f"Failed to clear robot positions after warehouse load: {e}")

            # Step 6: Persist metadata about the warehouse source
            try:
                with self.simulation_data_service._get_connection() as conn:
                    with conn.cursor() as cur:
                        # We persist metadata alongside the map. If there's no row yet,
                        # insert a new one with minimal fields and metadata. If there is,
                        # update metadata to reflect the new generation.
                        cur.execute(
                            """
                            DO $$
                            BEGIN
                                IF NOT EXISTS (SELECT 1 FROM warehouse_map) THEN
                                    INSERT INTO warehouse_map (width, height, grid_size, grid_data, source_csv_file, source_csv_hash)
                                    VALUES (%s, %s, %s, %s::jsonb, %s, %s);
                                ELSE
                                    UPDATE warehouse_map
                                    SET source_csv_file = %s,
                                        source_csv_hash = %s
                                    WHERE id = (SELECT id FROM warehouse_map ORDER BY id DESC LIMIT 1);
                                END IF;
                            END$$;
                            """,
                            (
                                self.warehouse_map.width,
                                self.warehouse_map.height,
                                self.warehouse_map.grid_size,
                                '{}',  # placeholder grid json when regenerating metadata only
                                self.warehouse_csv,
                                self._csv_hash or '',
                                self.warehouse_csv,
                                self._csv_hash or ''
                            ),
                        )
                        conn.commit()
                        log_success(f"Warehouse metadata persisted: file={self.warehouse_csv} hash={(self._csv_hash or '')[:8]}...")
            except Exception as e:
                log_warning(f"Failed to persist warehouse metadata: {e}")

            # Step 7: Get final statistics
            stats = self.simulation_data_service.get_inventory_statistics()
            log_info(f"📊 Final stats: {stats['total_items']} items across {stats['total_shelves']} shelves")

            log_info("🎉 Warehouse data generation completed successfully!")

            return WarehouseGenerationResult(
                navigation_graph=navigation_graph,
                shelves_created=shelves_created,
                inventory_created=inventory_created,
                success=True,
                errors=[]
            )

        except Exception as e:
            error_msg = f"Warehouse generation failed: {e}"
            log_error(f"❌ {error_msg}")
            errors.append(error_msg)

            return WarehouseGenerationResult(
                navigation_graph=GraphPersistenceResult(
                    nodes_persisted=0,
                    edges_persisted=0,
                    boxes_persisted=0,
                    cleared_existing=False
                ),
                shelves_created=0,
                inventory_created=0,
                success=False,
                errors=errors
            )


def main():
    """Main entry point for warehouse generation script."""
    parser = argparse.ArgumentParser(
        description="Generate warehouse data from CSV file",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python load_new_warehouse.py --warehouse extended_warehouse.csv
  python load_new_warehouse.py --warehouse custom_layout.csv

The script will:
1. Validate the warehouse CSV file
2. Generate navigation graph (conflict boxes, nodes, edges)
3. Create shelves from warehouse layout
4. Populate inventory with demo data
5. Ensure sample robot fleet exists (creates 3 robots if none found)
6. Clear robot positions to avoid stale placements
7. Export inventory status to CSV

Exit codes:
  0 - Success
  1 - Argument or file validation error
  2 - Warehouse generation failed
        """
    )

    parser.add_argument(
        "--warehouse",
        type=str,
        required=True,
        help="Warehouse CSV layout file (required)"
    )

    args = parser.parse_args()

    try:
        # Create warehouse generator
        generator = WarehouseGenerator(args.warehouse)

        # Generate warehouse data
        result = generator.generate_warehouse_data()

        if result.success:
            log_success("Warehouse generation completed successfully!")
            log_info(f"   📦 Navigation: {result.navigation_graph.boxes_persisted} boxes, "
                    f"{result.navigation_graph.nodes_persisted} nodes, "
                    f"{result.navigation_graph.edges_persisted} edges")
            log_info(f"   🏷️  Shelves: {result.shelves_created} created")
            log_info(f"   📋 Inventory: {result.inventory_created} items populated")
            return 0
        else:
            log_error("❌ Warehouse generation failed!")
            for error in result.errors:
                log_error(f"   • {error}")
            return 2

    except FileNotFoundError as e:
        log_error(f"Warehouse file not found: {e}")
        return 1
    except PermissionError as e:
        log_error(f"Permission error: {e}")
        return 1
    except ValueError as e:
        log_error(f"Invalid warehouse file: {e}")
        return 1
    except Exception as e:
        log_error(f"Unexpected error: {e}")
        return 3


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
