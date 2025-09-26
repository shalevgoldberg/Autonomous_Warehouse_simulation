#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Comprehensive Configuration Integration Demo

This demo shows how all refactored components work together with the centralized
configuration provider, demonstrating the complete system architecture.

Components demonstrated:
- ConfigurationProvider with multiple sources
- RobotAgent with configuration-driven parameters
- TaskHandler with configurable task execution
- JobsProcessor with configurable processing
- RobotController with configurable coordination
- LaneFollower and MotionExecutor with robot-specific config

Architecture Principles:
- SOLID principles throughout
- Dependency injection
- Interface-driven design
- Thread-safe operations
- Centralized configuration management
"""

import os
import sys
import time
import threading
import logging
from datetime import datetime, timedelta
from typing import List, Dict, Any

# Add the project root to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config.configuration_provider import ConfigurationProvider
from config.configuration_validator import ConfigurationValidatorImpl as ConfigurationValidator
from config.configuration_sources import (
    EnvironmentConfigurationSource, 
    FileConfigurationSource,
    DefaultConfigurationSource
)

from robot.robot_agent_lane_based import RobotAgent as RobotAgentLaneBased
from robot.impl.task_handler_impl import TaskHandlerImpl
from robot.impl.lane_follower_impl import LaneFollowerImpl
from robot.impl.motion_executor_impl import MotionExecutorImpl
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from robot.impl.state_holder_impl import StateHolderImpl
from robot.impl.path_planner_graph_impl import PathPlannerGraphImpl

from warehouse.impl.jobs_processor_impl import JobsProcessorImpl
from warehouse.impl.robot_controller_impl import RobotController
from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem as TransparentBiddingSystemImpl
from warehouse.impl.jobs_queue_impl import JobsQueueImpl
from warehouse.impl.json_order_source import JsonOrderSource

from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from simulation.mujoco_env import SimpleMuJoCoPhysics
from warehouse.map import WarehouseMap

from interfaces.task_handler_interface import Task, TaskType, TaskStatus
from interfaces.jobs_processor_interface import Order, OrderItem, Priority
from interfaces.bidding_system_interface import BiddingRound


def setup_logging():
    """Setup logging for the demo."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('demo_configuration_integration.log')
        ]
    )


def create_configuration_provider() -> ConfigurationProvider:
    """Create and configure the configuration provider."""
    print("\n=== Creating Configuration Provider ===")
    
    # Create configuration sources
    sources = [
        EnvironmentConfigurationSource(),
        FileConfigurationSource("config.yaml"),
        DefaultConfigurationSource()
    ]
    
    # Create validator
    validator = ConfigurationValidator()
    
    # Create provider
    provider = ConfigurationProvider(
        config_file="config.yaml",
        validator=validator
    )
    
    # Load and validate configuration
    provider.reload()
    
    # Set some environment variables for demonstration
    os.environ['WAREHOUSE_ROBOT_MAX_SPEED'] = '2.5'
    os.environ['WAREHOUSE_ROBOT_CONTROL_FREQUENCY'] = '15.0'
    os.environ['WAREHOUSE_TASK_ORDER_PROCESSING_INTERVAL'] = '2.0'
    os.environ['WAREHOUSE_SYSTEM_HEALTH_CHECK_INTERVAL'] = '15.0'
    
    # Reload to pick up environment variables
    provider.reload()
    
    print(f"Configuration loaded with {len(provider.errors)} errors")
    if provider.errors:
        print("Configuration errors:")
        for error in provider.errors:
            print(f"  - {error}")
    
    return provider


def create_warehouse_components(config_provider: ConfigurationProvider):
    """Create warehouse infrastructure components."""
    print("\n=== Creating Warehouse Components ===")
    
    # Create warehouse map
    warehouse_map = WarehouseMap(width=20, height=15)
    
    # Create physics engine
    physics = SimpleMuJoCoPhysics(warehouse_map=warehouse_map)
    
    # Create simulation data service
    simulation_data_service = SimulationDataServiceImpl(
        warehouse_map=warehouse_map
    )
    
    # Create order source
    order_source = JsonOrderSource("orders.json")
    
    # Create jobs queue
    jobs_queue = JobsQueueImpl()
    
    # Create bidding system
    bidding_system = TransparentBiddingSystemImpl()
    
    print("Warehouse components created successfully")
    
    return {
        'simulation_data_service': simulation_data_service,
        'order_source': order_source,
        'jobs_queue': jobs_queue,
        'bidding_system': bidding_system,
        'warehouse_map': warehouse_map,
        'physics': physics
    }


def create_robot_agent(robot_id: str, config_provider: ConfigurationProvider, 
                      components: Dict[str, Any]) -> RobotAgentLaneBased:
    """Create a robot agent with configuration-driven parameters."""
    print(f"\n=== Creating Robot Agent {robot_id} ===")
    
    # Get robot configuration
    robot_config = config_provider.get_robot_config(robot_id)
    print(f"Robot {robot_id} configuration:")
    print(f"  Max Speed: {robot_config.max_speed} m/s")
    print(f"  Control Frequency: {robot_config.control_frequency} Hz")
    print(f"  Motion Frequency: {robot_config.motion_frequency} Hz")
    print(f"  Corner Speed: {robot_config.corner_speed} m/s")
    print(f"  Bay Approach Speed: {robot_config.bay_approach_speed} m/s")
    print(f"  Picking Duration: {robot_config.picking_duration} s")
    print(f"  Dropping Duration: {robot_config.dropping_duration} s")
    
    # Create robot agent
    robot = RobotAgentLaneBased(
        physics=components['physics'],
        config_provider=config_provider,
        robot_id=robot_id,
        simulation_data_service=components['simulation_data_service']
    )
    
    print(f"Robot {robot_id} created successfully")
    return robot


def create_jobs_processor(config_provider: ConfigurationProvider, 
                         components: Dict[str, Any]) -> JobsProcessorImpl:
    """Create jobs processor with configuration-driven parameters."""
    print("\n=== Creating Jobs Processor ===")
    
    # Get task configuration
    task_config = config_provider.get_task_config()
    print("Task configuration:")
    print(f"  Max Concurrent Orders: {task_config.max_concurrent_orders}")
    print(f"  Order Processing Interval: {task_config.order_processing_interval} s")
    print(f"  Inventory Allocation Timeout: {task_config.inventory_allocation_timeout} s")
    print(f"  Bidding Timeout: {task_config.bidding_timeout} s")
    print(f"  Task Timeout: {task_config.task_timeout} s")
    
    # Create jobs processor
    jobs_processor = JobsProcessorImpl(
        order_source=components['order_source'],
        simulation_data_service=components['simulation_data_service'],
        jobs_queue=components['jobs_queue'],
        config_provider=config_provider
    )
    
    print("Jobs processor created successfully")
    return jobs_processor


def create_robot_controller(config_provider: ConfigurationProvider, 
                           components: Dict[str, Any], 
                           robots: List[RobotAgentLaneBased]) -> RobotController:
    """Create robot controller with configuration-driven parameters."""
    print("\n=== Creating Robot Controller ===")
    
    # Get system configuration
    system_config = config_provider.get_system_config()
    print("System configuration:")
    print(f"  Health Check Interval: {system_config.health_check_interval} s")
    print(f"  Metrics Collection Interval: {system_config.metrics_collection_interval} s")
    print(f"  Log Level: {system_config.log_level}")
    
    # Create robot controller
    controller = RobotController(
        jobs_queue=components['jobs_queue'],
        bidding_system=components['bidding_system'],
        robot_pool=robots,
        config_provider=config_provider
    )
    
    print("Robot controller created successfully")
    return controller


def create_sample_orders():
    """Create sample orders for demonstration."""
    print("\n=== Creating Sample Orders ===")
    
    # Create sample orders
    orders = [
        Order(
            order_id="order_001",
            items=[OrderItem(item_id="item_A", quantity=2)],
            scheduled_time=datetime.now(),
            priority=Priority.HIGH,
            customer_id="customer_001"
        ),
        Order(
            order_id="order_002",
            items=[OrderItem(item_id="item_B", quantity=1)],
            scheduled_time=datetime.now(),
            priority=Priority.NORMAL,
            customer_id="customer_002"
        ),
        Order(
            order_id="order_003",
            items=[
                OrderItem(item_id="item_A", quantity=1),
                OrderItem(item_id="item_C", quantity=3)
            ],
            scheduled_time=datetime.now(),
            priority=Priority.LOW,
            customer_id="customer_003"
        )
    ]
    
    # Save to JSON file
    import json
    orders_data = []
    for order in orders:
        order_dict = {
            'order_id': order.order_id,
            'items': [{'item_id': item.item_id, 'quantity': item.quantity} for item in order.items],
            'scheduled_time': order.scheduled_time.isoformat(),
            'priority': order.priority.value,
            'customer_id': order.customer_id
        }
        orders_data.append(order_dict)
    
    with open('orders.json', 'w') as f:
        json.dump(orders_data, f, indent=2)
    
    print(f"Created {len(orders)} sample orders in orders.json")
    return orders


def demonstrate_configuration_updates(config_provider: ConfigurationProvider, 
                                    jobs_processor: JobsProcessorImpl,
                                    robot_controller: RobotController):
    """Demonstrate runtime configuration updates."""
    print("\n=== Demonstrating Configuration Updates ===")
    
    # Show current configuration
    print("Current configuration:")
    task_config = config_provider.get_task_config()
    system_config = config_provider.get_system_config()
    print(f"  Order Processing Interval: {task_config.order_processing_interval} s")
    print(f"  Health Check Interval: {system_config.health_check_interval} s")
    
    # Update configuration at runtime
    print("\nUpdating configuration at runtime...")
    config_provider.set_value("task.order_processing_interval", 1.5)
    config_provider.set_value("system.health_check_interval", 10.0)
    
    # Update components
    jobs_processor.update_config_from_provider()
    robot_controller.update_config_from_provider()
    
    # Show updated configuration
    print("Updated configuration:")
    task_config = config_provider.get_task_config()
    system_config = config_provider.get_system_config()
    print(f"  Order Processing Interval: {task_config.order_processing_interval} s")
    print(f"  Health Check Interval: {system_config.health_check_interval} s")
    
    print("Configuration updates applied successfully")


def demonstrate_system_operation(robots: List[RobotAgentLaneBased], 
                               jobs_processor: JobsProcessorImpl,
                               robot_controller: RobotController):
    """Demonstrate the complete system operation."""
    print("\n=== Demonstrating System Operation ===")
    
    try:
        # Start all components
        print("Starting system components...")
        
        # Initialize robots
        for robot in robots:
            robot.initialize_position()
            status = robot.get_status()
            print(f"Robot {robot.robot_id} initialized at position {status['position']}")
        
        # Start jobs processor
        jobs_processor.start_processing()
        print("Jobs processor started")
        
        # Start robot controller
        robot_controller.start()
        print("Robot controller started")
        
        # Let the system run for a while
        print("System running for 10 seconds...")
        time.sleep(10)
        
        # Show system status
        print("\nSystem Status:")
        
        # Jobs processor status
        stats = jobs_processor.get_processing_stats()
        print(f"  Jobs Processor:")
        print(f"    Total Orders Processed: {stats.total_orders_processed}")
        print(f"    Successful Orders: {stats.successful_orders}")
        print(f"    Failed Orders: {stats.failed_orders}")
        print(f"    Total Tasks Created: {stats.total_tasks_created}")
        
        # Robot controller status
        controller_status = robot_controller.get_controller_status()
        print(f"  Robot Controller:")
        print(f"    Running: {controller_status['running']}")
        print(f"    Total Rounds Processed: {controller_status['total_rounds_processed']}")
        print(f"    Total Tasks Assigned: {controller_status['total_tasks_assigned']}")
        
        # Robot statuses
        robot_statuses = robot_controller.get_robot_status_summary()
        print(f"  Robots:")
        for status in robot_statuses:
            print(f"    {status['robot_id']}: {status['status']}")
        
        # Stop components
        print("\nStopping system components...")
        jobs_processor.stop_processing()
        robot_controller.stop()
        
        print("System demonstration completed successfully")
        
    except Exception as e:
        print(f"Error during system operation: {e}")
        import traceback
        traceback.print_exc()


def demonstrate_error_handling(config_provider: ConfigurationProvider):
    """Demonstrate configuration error handling."""
    print("\n=== Demonstrating Error Handling ===")
    
    # Try to set invalid configuration values
    print("Setting invalid configuration values...")
    config_provider.set_value("robot.max_speed", -1.0)  # Invalid negative speed
    config_provider.set_value("robot.control_frequency", 0.0)  # Invalid zero frequency
    
    # Validate configuration
    errors = config_provider.validate()
    print(f"Configuration validation found {len(errors)} errors:")
    for error in errors:
        print(f"  - {error}")
    
    # Show how to fix errors
    print("\nFixing configuration errors...")
    config_provider.set_value("robot.max_speed", 2.0)  # Valid speed
    config_provider.set_value("robot.control_frequency", 10.0)  # Valid frequency
    
    # Validate again
    errors = config_provider.validate()
    print(f"After fixes: {len(errors)} errors remaining")
    
    print("Error handling demonstration completed")


def main():
    """Main demonstration function."""
    print("=== Autonomous Warehouse Configuration Integration Demo ===")
    print("This demo shows how all components work together with centralized configuration")
    
    # Setup logging
    setup_logging()
    
    try:
        # Create configuration provider
        config_provider = create_configuration_provider()
        
        # Create warehouse components
        components = create_warehouse_components(config_provider)
        
        # Create sample orders
        create_sample_orders()
        
        # Create robot agents
        robots = []
        for i in range(2):
            robot_id = f"robot_{i+1}"
            robot = create_robot_agent(robot_id, config_provider, components)
            robots.append(robot)
        
        # Create jobs processor
        jobs_processor = create_jobs_processor(config_provider, components)
        
        # Create robot controller
        robot_controller = create_robot_controller(config_provider, components, robots)
        
        # Demonstrate configuration updates
        demonstrate_configuration_updates(config_provider, jobs_processor, robot_controller)
        
        # Demonstrate error handling
        demonstrate_error_handling(config_provider)
        
        # Demonstrate system operation
        demonstrate_system_operation(robots, jobs_processor, robot_controller)
        
        print("\n=== Demo Completed Successfully ===")
        print("All components are now using centralized configuration management")
        print("The system follows SOLID principles and is fully configurable")
        
    except Exception as e:
        print(f"Demo failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main()) 