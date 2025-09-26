#!/usr/bin/env python3
"""
End-to-End Multi-Robot Warehouse Demo

This demo demonstrates the COMPLETE autonomous warehouse workflow:
External Orders → JobsProcessor → JobsQueue → Bidding → Robot Assignment → Execution

Features:
- External order input (JSON file simulation)
- JobsProcessor converting orders to robot tasks
- Multi-robot coordination with bidding system
- Conflict box coordination for safe navigation
- Real-time database integration
- Complete task lifecycle monitoring
- Professional visualization

⚠️  DEMO CONFIGURATION: Lane tolerance set to 3.0m (too soft for production!)
    This allows tasks to complete despite navigation issues. For production,
    use 0.1-0.3m tolerance for safe warehouse operation.

Run with: python demo_end_to_end_multi_robot.py
"""
import sys
import os
import time
import threading
from typing import List, Optional, Dict, Any
import argparse
from dataclasses import dataclass
from pathlib import Path
import tempfile
import json
import subprocess
import logging

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent_lane_based import RobotAgent
from interfaces.configuration_interface import RobotConfig
from interfaces.task_handler_interface import Task, TaskType
from interfaces.path_planner_interface import Cell
from interfaces.visualization_interface import IVisualization
from simulation.mujoco_visualization import MujocoVisualization
from simulation.multi_robot_mujoco_visualization import MultiRobotMujocoVisualization
from simulation.visualization_thread import VisualizationThread
from interfaces.lane_follower_interface import LaneFollowingConfig
from config.configuration_provider import ConfigurationProvider
from simulation.simulation_data_service_impl import SimulationDataServiceImpl

# Order processing components
from warehouse.impl.json_order_source import JsonOrderSource
from warehouse.impl.jobs_processor_impl import JobsProcessorImpl
from warehouse.impl.jobs_queue_impl import JobsQueueImpl

# Multi-robot coordination
from warehouse.impl.robot_controller_impl import RobotController
from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem


@dataclass
class RobotInstance:
    """Container for robot instance and its metadata."""
    robot: RobotAgent
    config: RobotConfig
    start_position: tuple
    robot_id: str
    status: str = "initializing"


@dataclass
class OrderMetrics:
    """Track order processing metrics."""
    orders_created: int = 0
    orders_processed: int = 0
    tasks_created: int = 0
    tasks_assigned: int = 0
    tasks_completed: int = 0


class EndToEndMultiRobotDemo:
    """
    Complete End-to-End Multi-Robot Warehouse Demo.

    This demo shows the full autonomous warehouse workflow:
    1. External Orders (JSON files)
    2. JobsProcessor (order → tasks)
    3. JobsQueue (task distribution)
    4. Bidding System (parallel robot bidding)
    5. Robot Controller (task assignment)
    6. Robot Execution (complete lifecycle)
    7. Database Integration (inventory, navigation, conflict boxes)
    """

    def __init__(self, warehouse_csv: str = "sample_warehouse.csv", robot_count: int = 2):
        """Initialize the complete end-to-end demo."""
        print("🚀 Initializing End-to-End Multi-Robot Warehouse Demo")
        print(f"   🤖 Robot Count: {robot_count}")

        # Validate robot count
        if robot_count < 1 or robot_count > 4:
            raise ValueError("Robot count must be between 1 and 4 for demo purposes")

        self.robot_count = robot_count
        self.robots: List[RobotInstance] = []
        self.metrics = OrderMetrics()

        # Temporary directory for external orders
        self.temp_dir = Path(tempfile.mkdtemp(prefix="warehouse_demo_"))
        self.orders_file = self.temp_dir / "orders.json"
        self.external_script = self.temp_dir / "create_order.py"

        # Load warehouse
        self.warehouse_map = WarehouseMap(csv_file=warehouse_csv)
        print(f"   ✅ Warehouse loaded: {self.warehouse_map.width}x{self.warehouse_map.height}")

        # Create configuration provider
        self.config_provider = ConfigurationProvider()
        print("   ✅ Configuration provider initialized")

        # Create simulation data service
        self.simulation_data_service = SimulationDataServiceImpl(
            self.warehouse_map,
            pool_size=max(10, 3 * robot_count)
        )
        print("   ✅ Simulation data service initialized")

        # Setup navigation graph and inventory
        self._setup_database()

        # Create external order system
        self._create_external_order_system()

        # Create job processing components
        self._create_job_processing_system()

        # Create robots and coordination
        self._create_robots_and_coordination()

        # Visualization
        self.visualization: Optional[IVisualization] = None
        self.visualization_thread: Optional[VisualizationThread] = None

        print("🎯 End-to-End Demo ready!")
        print("   📋 Complete workflow: External Orders → Jobs → Bidding → Robots → Execution")

    def _setup_database(self) -> None:
        """Setup database with navigation graph and inventory."""
        print("   🗄️  Setting up database...")

        try:
            # Create navigation graph
            result = self.simulation_data_service.persist_navigation_graph_from_csv(
                Path("sample_warehouse.csv"), clear_existing=True
            )
            print(f"      ✅ Navigation graph: {result.nodes_persisted} nodes, "
                  f"{result.edges_persisted} edges, {result.boxes_persisted} conflict boxes")

            # Create shelves and inventory
            shelves_created = self.simulation_data_service.create_shelves_from_map(clear_existing=True)
            print(f"      ✅ Created {shelves_created} shelves from warehouse map")

            # Demo inventory for orders
            demo_inventory = [
                {'item_id': 'laptop_001', 'name': 'Gaming Laptop', 'shelf_id': 'shelf_3_3', 'quantity': 5},
                {'item_id': 'phone_001', 'name': 'Smartphone', 'shelf_id': 'shelf_4_3', 'quantity': 10},
                {'item_id': 'tablet_001', 'name': 'Tablet', 'shelf_id': 'shelf_6_3', 'quantity': 8},
                {'item_id': 'headphones_001', 'name': 'Wireless Headphones', 'shelf_id': 'shelf_7_3', 'quantity': 15},
                {'item_id': 'keyboard_001', 'name': 'Mechanical Keyboard', 'shelf_id': 'shelf_3_6', 'quantity': 12},
                {'item_id': 'mouse_001', 'name': 'Gaming Mouse', 'shelf_id': 'shelf_4_6', 'quantity': 20},
                {'item_id': 'monitor_001', 'name': '4K Monitor', 'shelf_id': 'shelf_6_6', 'quantity': 6},
                {'item_id': 'speaker_001', 'name': 'Bluetooth Speaker', 'shelf_id': 'shelf_7_6', 'quantity': 9},
                {'item_id': 'camera_001', 'name': 'Action Camera', 'shelf_id': 'shelf_3_9', 'quantity': 7},
                {'item_id': 'drone_001', 'name': 'Quadcopter Drone', 'shelf_id': 'shelf_4_9', 'quantity': 4},
            ]

            inventory_created = self.simulation_data_service.populate_inventory(demo_inventory)
            print(f"      ✅ Populated {inventory_created} inventory items")

            stats = self.simulation_data_service.get_inventory_statistics()
            print(f"      📊 Inventory: {stats['total_items']} items across {stats['total_shelves']} shelves")

        except Exception as e:
            print(f"      ⚠️  Database setup warning: {e}")

    def _create_external_order_system(self) -> None:
        """Create external order input system."""
        print("   📝 Setting up external order system...")

        # Initialize empty orders file
        self.orders_file.write_text("[]", encoding='utf-8')

        # Create external order creation script
        script = r"""
import json, sys, datetime
file_path = sys.argv[1]
item_id = sys.argv[2]
quantity = int(sys.argv[3]) if len(sys.argv) > 3 else 1

# Generate order ID
import uuid
order_id = f"order_{uuid.uuid4().hex[:8]}"

# Create order with current timestamp
now = datetime.datetime.now().isoformat()
order = {
    "order_id": order_id,
    "items": [{"item_id": item_id, "quantity": quantity}],
    "scheduled_time": now,
    "priority": "normal",
    "metadata": {"source": "external_system", "customer_id": f"customer_{order_id}"}
}

# Load existing orders
try:
    with open(file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
except:
    data = []

# Add new order
data.append(order)

# Save back to file
with open(file_path, 'w', encoding='utf-8') as f:
    json.dump(data, f, indent=2)

print(f"Created order {order_id} for {quantity}x {item_id}")
"""

        self.external_script.write_text(script, encoding='utf-8')
        print(f"      ✅ External order script: {self.external_script}")

    def _create_job_processing_system(self) -> None:
        """Create job processing components."""
        print("   🔄 Setting up job processing system...")

        # Order source (reads from JSON file)
        self.order_source = JsonOrderSource(str(self.orders_file))
        self.order_source.connect()
        print("      ✅ Order source connected")

        # Jobs queue (distributes tasks to robots)
        self.jobs_queue = JobsQueueImpl()
        print("      ✅ Jobs queue initialized")

        # Jobs processor (converts orders to tasks)
        self.jobs_processor = JobsProcessorImpl(
            order_source=self.order_source,
            simulation_data_service=self.simulation_data_service,
            jobs_queue=self.jobs_queue,
        )
        print("      ✅ Jobs processor initialized")

    def _create_robots_and_coordination(self) -> None:
        """Create robots and coordination system."""
        print("   🤖 Setting up robots and coordination...")

        # Define starting positions
        start_positions = [
            (1, 1), (1, 2), (1, 3), (1, 4)
        ]

        for i in range(self.robot_count):
            robot_id = "02d"
            start_x, start_y = start_positions[i]

            # Create robot configuration
            robot_config = RobotConfig(
                robot_id=robot_id,
                max_speed=2.0,
                control_frequency=10.0,
                motion_frequency=100.0,
                cell_size=1.0,
                lane_tolerance=3.0,  # DEMO ONLY
                corner_speed=1.0,
                bay_approach_speed=0.5,
                conflict_box_lock_timeout=5.0,
                conflict_box_heartbeat_interval=1.0,
                max_linear_velocity=2.0,
                max_angular_velocity=2.0,
                movement_speed=2.0,
                wheel_base=0.5,
                wheel_radius=0.1,
                picking_duration=2.0,
                dropping_duration=2.0,
                position_tolerance=0.1,
                emergency_stop_distance=0.5,
                stall_recovery_timeout=10.0
            )

            # Create dedicated physics engine per robot
            robot_physics = SimpleMuJoCoPhysics(self.warehouse_map)

            # Register robot in database
            try:
                with self.simulation_data_service._get_connection() as conn:
                    with conn.cursor() as cur:
                        cur.execute(
                            "INSERT INTO robots (robot_id, name) VALUES (%s, %s) ON CONFLICT (robot_id) DO NOTHING",
                            (robot_id, robot_id.replace('_', ' ').title()),
                        )
                        conn.commit()
            except Exception:
                pass

            # Create robot agent
            robot = RobotAgent(
                physics=robot_physics,
                config_provider=self.config_provider,
                robot_id=robot_id,
                simulation_data_service=self.simulation_data_service
            )

            # Configure lane follower
            robot.lane_follower.set_config(
                LaneFollowingConfig(
                    lane_tolerance=3.0,  # DEMO ONLY
                    max_speed=robot_config.max_speed,
                    corner_speed=robot_config.corner_speed,
                    bay_approach_speed=robot_config.bay_approach_speed,
                )
            )

            # Create robot instance
            robot_instance = RobotInstance(
                robot=robot,
                config=robot_config,
                start_position=(start_x, start_y),
                robot_id=robot_id
            )

            self.robots.append(robot_instance)
            print(f"      ✅ Created {robot_id} at position ({start_x}, {start_y})")

        # Create bidding system and robot controller
        self.bidding_system = TransparentBiddingSystem(config_provider=self.config_provider)

        # Adapter to match controller expectations
        class RobotAdapter:
            def __init__(self, robot_instance):
                self._robot = robot_instance.robot
                self.config = type("Cfg", (), {"robot_id": robot_instance.robot_id})()
            def __getattr__(self, name):
                return getattr(self._robot, name)

        adapted_robots = [RobotAdapter(r) for r in self.robots]

        self.robot_controller = RobotController(
            jobs_queue=self.jobs_queue,
            bidding_system=self.bidding_system,
            robot_pool=adapted_robots,
            polling_interval=0.5
        )

        print(f"   ✅ Created {len(self.robots)} robots with coordination system")

    def create_external_order(self, item_id: str, quantity: int = 1) -> str:
        """Create an external order using the external script."""
        try:
            cmd = [
                "python",
                str(self.external_script),
                str(self.orders_file),
                item_id,
                str(quantity)
            ]
            result = subprocess.check_output(cmd, text=True)
            # Extract order ID from output like "Created order order_a1b2c3d4 for 1x laptop_001"
            lines = result.strip().split('\n')
            for line in lines:
                if 'Created order' in line and 'for' in line:
                    parts = line.split()
                    if len(parts) >= 4:
                        order_id = parts[2]  # "order_a1b2c3d4"
                        break
            self.metrics.orders_created += 1
            print(f"   📦 External order created: {order_id}")
            return order_id
        except Exception as e:
            print(f"   ❌ Failed to create external order: {e}")
            return ""

    def process_orders_once(self) -> int:
        """Process pending orders into tasks."""
        try:
            result = self.jobs_processor.process_next_order()
            if result:
                self.metrics.orders_processed += 1
                task_count = len(result.tasks_created) if isinstance(result.tasks_created, list) else 0
                self.metrics.tasks_created += task_count
                print(f"   🔄 Order processed: {result.order_id} → {task_count} tasks")
                return task_count
            return 0
        except Exception as e:
            print(f"   ❌ Order processing failed: {e}")
            return 0

    def start_demo(self) -> None:
        """Start the complete demo."""
        print("\n🎬 Starting End-to-End Multi-Robot Demo")

        # Configure logging
        self._configure_logging()

        # Start robots
        for robot_instance in self.robots:
            robot_instance.robot.start()
            robot_instance.status = "running"
            print(f"   ✅ Started {robot_instance.robot_id}")

        # Position robots
        for robot_instance in self.robots:
            start_x, start_y = robot_instance.start_position
            world_pos = self.warehouse_map.grid_to_world(start_x, start_y)
            robot_instance.robot.physics.reset_robot_position(world_pos[0], world_pos[1], 0.0)
            print(f"   ✅ Positioned {robot_instance.robot_id} at ({start_x}, {start_y})")

        # Start visualization
        try:
            state_holders = {r.robot_id: r.robot.state_holder for r in self.robots}
            self.visualization = MultiRobotMujocoVisualization(state_holders, self.warehouse_map)
            self.visualization.initialize()
            self.visualization_thread = VisualizationThread(self.visualization, fps=30.0)
            self.visualization_thread.start()
            print("   ✅ Multi-robot visualization started")
        except Exception as e:
            print(f"   ⚠️ Visualization failed: {e}")

        # Start robot controller
        self.robot_controller.start()
        print("   ✅ Robot controller started")

        print("   🎯 Demo ready for orders!")

    def _configure_logging(self) -> None:
        """Configure logging for robots."""
        try:
            for robot_instance in self.robots:
                lane_logger = logging.getLogger(f"LaneFollower.{robot_instance.robot_id}")
                lane_logger.setLevel(logging.INFO)
                if not any(isinstance(h, logging.StreamHandler) for h in lane_logger.handlers):
                    handler = logging.StreamHandler()
                    handler.setLevel(logging.INFO)
                    handler.setFormatter(logging.Formatter("[%(levelname)s] [%(name)s] %(message)s"))
                    lane_logger.addHandler(handler)
        except Exception:
            pass

    def run_demo_workflow(self, duration_minutes: float = 3.0) -> None:
        """Run the complete demo workflow."""
        print(f"\n⏳ Running End-to-End Demo for {duration_minutes} minutes")
        print("   📋 Complete workflow demonstration:")
        print("   1. 📦 External orders created")
        print("   2. 🔄 Orders processed into tasks")
        print("   3. 🤖 Robots bid for tasks")
        print("   4. 🎮 Controller assigns tasks")
        print("   5. 🚀 Robots execute complete lifecycle")

        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)
        last_status_time = 0
        last_order_time = 0

        # Available items for orders
        available_items = [
            'laptop_001', 'phone_001', 'tablet_001', 'headphones_001',
            'keyboard_001', 'mouse_001', 'monitor_001', 'speaker_001'
        ]

        while time.time() < end_time:
            current_time = time.time()

            # Create external orders every 30 seconds
            if current_time - last_order_time >= 30.0:
                item_id = available_items[self.metrics.orders_created % len(available_items)]
                self.create_external_order(item_id)
                last_order_time = current_time

                # Let order source detect the file change
                self.order_source.refresh()

                # Process orders into tasks
                tasks_created = self.process_orders_once()
                if tasks_created > 0:
                    queue_stats = self.jobs_queue.get_queue_stats()
                    print(f"   📋 Tasks in queue: {queue_stats.get('queue_size', 0)}")

            # Print status every 15 seconds
            if current_time - last_status_time >= 15.0:
                self._print_status_update(current_time - start_time)
                last_status_time = current_time

            time.sleep(2.0)

        print(f"\n🎉 Demo completed after {duration_minutes} minutes!")
        self._print_final_metrics()

    def _print_status_update(self, elapsed_time: float) -> None:
        """Print comprehensive status update."""
        print(f"\n--- Status Update at {elapsed_time:.1f}s ---")

        # Order processing status
        print("   📦 Orders: "
              f"Created={self.metrics.orders_created}, "
              f"Processed={self.metrics.orders_processed}, "
              f"Pending={self.metrics.orders_created - self.metrics.orders_processed}")

        # Queue status
        queue_stats = self.jobs_queue.get_queue_stats()
        print(f"   📋 Queue: {queue_stats.get('queue_size', 0)} tasks waiting")

        # Controller status
        controller_status = self.robot_controller.get_controller_status()
        print(f"   🎮 Controller: {controller_status['total_rounds_processed']} rounds, "
              f"{controller_status['total_tasks_assigned']} tasks assigned")

        # Robot statuses
        for robot_instance in self.robots:
            try:
                robot_status = robot_instance.robot.get_status()
                position = robot_status.get('position', [0.0, 0.0, 0.0])
                task_status = robot_status.get('task_status')

                if task_status and hasattr(task_status, 'operational_status'):
                    operational_status = task_status.operational_status.value
                    progress = getattr(task_status, 'progress', 0.0)
                else:
                    operational_status = 'idle'
                    progress = 0.0

                # Get battery level from robot status
                battery_level = robot_status.get('battery_level', 0.0)

                # Get conflict box status
                acquired_boxes = robot_status.get('acquired_conflict_boxes', [])
                pending_box = robot_status.get('pending_conflict_box')

                # Format conflict box info
                if acquired_boxes:
                    conflict_info = f"Locks: {acquired_boxes}"
                else:
                    conflict_info = "Locks: None"

                if pending_box:
                    conflict_info += f" | Pending: {pending_box}"
                else:
                    conflict_info += " | Pending: None"

                print(f"   🤖 {robot_instance.robot_id}: {operational_status} | "
                      f"Progress: {progress:.1%} | "
                      f"Position: ({position[0]:.1f}, {position[1]:.1f}) | "
                      f"Battery: {battery_level:.1%} | "
                      f"{conflict_info}")
            except Exception as e:
                print(f"   🤖 {robot_instance.robot_id}: Error - {e}")

    def _print_final_metrics(self) -> None:
        """Print final demo metrics."""
        print("\n📊 FINAL DEMO METRICS:")
        print(f"   📦 Orders Created: {self.metrics.orders_created}")
        print(f"   🔄 Orders Processed: {self.metrics.orders_processed}")
        print(f"   📋 Tasks Created: {self.metrics.tasks_created}")

        controller_status = self.robot_controller.get_controller_status()
        print(f"   🎮 Bidding Rounds: {controller_status['total_rounds_processed']}")
        print(f"   🤖 Tasks Assigned: {controller_status['total_tasks_assigned']}")

        # Calculate efficiency
        if self.metrics.orders_created > 0:
            processing_rate = self.metrics.orders_processed / self.metrics.orders_created * 100
            print(f"   📈 Order Processing Rate: {processing_rate:.1f}%")

        # KPI Overview (Phase 5)
        try:
            cfg = self.config_provider
            export_enabled = cfg.get_value("kpi.export_csv_enabled", True).value
            export_path = cfg.get_value("kpi.export_csv_path", "kpi_results.csv").value
        except Exception:
            export_enabled = True
            export_path = "kpi_results.csv"

        try:
            overview = self.simulation_data_service.get_kpi_overview()
            print("   📊 KPI Overview:")
            print(f"      - Tasks Completed: {overview.get('tasks_completed', 0)}")
            print(f"      - Task Success Rate: {overview.get('task_success_rate', 0.0):.1f}%")
            print(f"      - Avg Task Time: {overview.get('avg_task_time_seconds', 0.0):.2f}s")
            if export_enabled:
                if self.simulation_data_service.export_kpi_overview_csv(overview, export_path):
                    print(f"   💾 KPI CSV exported: {export_path}")
                else:
                    print("   ⚠️  KPI CSV export failed (see logs)")
        except Exception as e:
            print(f"   ⚠️  KPI overview unavailable: {e}")

        print("   ✅ Complete End-to-End Workflow Demonstrated!")

    def stop_demo(self) -> None:
        """Stop the complete demo."""
        print("\n🛑 Stopping End-to-End Demo")

        # Best-effort: clean up charging/idle bay locks for all robots before stopping threads
        try:
            for robot_instance in self.robots:
                try:
                    handler = getattr(robot_instance.robot, 'task_handler', None)
                    if handler is None:
                        continue
                    locked_id = getattr(handler, '_locked_bay_id', None)
                    if not locked_id:
                        continue
                    # Route releases based on lock type to keep caches consistent
                    if isinstance(locked_id, str) and locked_id.startswith('charge_'):
                        manager = getattr(handler, 'charging_station_manager', None)
                        if manager is not None:
                            try:
                                manager.release_charging_station(locked_id, robot_instance.robot_id)
                            except Exception:
                                try:
                                    handler.simulation_data_service.release_bay_lock(locked_id, robot_instance.robot_id)
                                except Exception:
                                    pass
                        else:
                            try:
                                handler.simulation_data_service.release_bay_lock(locked_id, robot_instance.robot_id)
                            except Exception:
                                pass
                    else:
                        try:
                            handler.simulation_data_service.release_bay_lock(locked_id, robot_instance.robot_id)
                        except Exception:
                            pass
                    try:
                        handler._locked_bay_id = None
                    except Exception:
                        pass
                except Exception:
                    pass
        except Exception:
            pass

        # Stop robot controller
        try:
            self.robot_controller.stop()
            print("   ✅ Robot controller stopped")
        except Exception as e:
            print(f"   ⚠️  Error stopping controller: {e}")

        # Stop robots
        for robot_instance in self.robots:
            try:
                robot_instance.robot.stop()
                robot_instance.status = "stopped"
                print(f"   ✅ Stopped {robot_instance.robot_id}")
            except Exception as e:
                print(f"   ⚠️  Error stopping {robot_instance.robot_id}: {e}")

        # Shutdown charging station managers to clear any internal assignments and caches
        try:
            for robot_instance in self.robots:
                try:
                    manager = getattr(robot_instance.robot, 'charging_station_manager', None)
                    if manager is not None and hasattr(manager, 'shutdown'):
                        manager.shutdown()
                except Exception:
                    pass
        except Exception:
            pass

        # Stop visualization
        try:
            if self.visualization_thread:
                self.visualization_thread.stop()
                self.visualization_thread = None
            if self.visualization:
                self.visualization.shutdown()
        except Exception:
            pass

        # Close connections
        try:
            self.order_source.disconnect()
            self.simulation_data_service.close()
        except Exception:
            pass

        # Cleanup temp files
        try:
            import shutil
            shutil.rmtree(self.temp_dir, ignore_errors=True)
        except Exception:
            pass

        print("   ✅ Demo stopped cleanly")


def main():
    """Main entry point."""
    print("🤖 End-to-End Multi-Robot Warehouse Demo")
    print("=" * 60)
    print("Complete autonomous warehouse workflow demonstration")
    print("External Orders → JobsProcessor → JobsQueue → Bidding → Robots")
    print()

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", type=int, default=2, help="Number of robots (1-4)")
    parser.add_argument("--duration", type=float, default=3.0, help="Demo duration in minutes")
    args = parser.parse_args()

    try:
        # Create and run demo
        demo = EndToEndMultiRobotDemo(robot_count=args.robots)

        demo.start_demo()

        # Run the complete workflow
        demo.run_demo_workflow(duration_minutes=args.duration)

    except KeyboardInterrupt:
        print("\n⚠️  Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            demo.stop_demo()
        except:
            pass


if __name__ == "__main__":
    main()
