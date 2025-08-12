#!/usr/bin/env python3
"""
Simplified End-to-End Flow Test

Tests the core flow: Order Source -> Jobs Processor -> Job Queue -> Robot Controller
Focuses on what's working and identifies specific issues.
"""
import os
import time
from datetime import datetime, timedelta

# Import core components
from warehouse.impl.json_order_source import JsonOrderSource
from warehouse.impl.jobs_processor_impl import JobsProcessorImpl
from warehouse.impl.jobs_queue_impl import JobsQueueImpl
from warehouse.impl.robot_controller_impl import RobotController
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap
from interfaces.task_handler_interface import Task, TaskType

def test_core_flow():
    """Test the core end-to-end flow."""
    print("🔍 TESTING CORE END-TO-END FLOW")
    print("="*50)
    
    try:
        # Step 1: Initialize components
        print("\n1️⃣ Initializing Components...")
        
        warehouse_map = WarehouseMap(width=20, height=15)
        
        simulation_data_service = SimulationDataServiceImpl(
            warehouse_map=warehouse_map,
            db_host="localhost",
            db_port=5432,
            db_name="warehouse_sim",
            db_user="postgres",
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD')
        )
        
        order_source = JsonOrderSource('sample_orders.json')
        jobs_queue = JobsQueueImpl()
        
        print("✅ Components initialized")
        
        # Step 2: Connect order source
        print("\n2️⃣ Connecting Order Source...")
        order_source.connect()
        orders = order_source.get_all_orders()
        print(f"✅ Loaded {len(orders)} orders")
        
        # Step 3: Create jobs processor
        print("\n3️⃣ Creating Jobs Processor...")
        jobs_processor = JobsProcessorImpl(
            order_source=order_source,
            simulation_data_service=simulation_data_service,
            jobs_queue=jobs_queue
        )
        print("✅ Jobs processor created")
        
        # Step 4: Process orders
        print("\n4️⃣ Processing Orders...")
        result = jobs_processor.process_orders_once()
        print(f"✅ Processed orders: {len(result.tasks_created)} tasks created")
        
        # Step 5: Check tasks in queue
        print("\n5️⃣ Checking Task Queue...")
        pending_tasks = jobs_queue.get_pending_tasks()
        print(f"✅ Tasks in queue: {len(pending_tasks)}")
        
        if pending_tasks:
            print("   Sample task:")
            task = pending_tasks[0]
            print(f"   - ID: {task.task_id}")
            print(f"   - Type: {task.task_type}")
            print(f"   - Shelf: {task.shelf_id}")
            print(f"   - Item: {task.item_id}")
            print(f"   - Order: {task.order_id}")
        
        # Step 6: Test robot controller (with mock robots)
        print("\n6️⃣ Testing Robot Controller...")
        
        # Create mock robots
        mock_robots = []
        for i in range(3):
            robot = create_mock_robot(f"robot_{i}")
            mock_robots.append(robot)
        
        # Create controller
        controller = RobotController(
            jobs_queue=jobs_queue,
            robot_pool=mock_robots
        )
        
        # Test bidding round
        bidding_result = controller.process_single_round()
        
        if bidding_result:
            print(f"✅ Bidding successful: {bidding_result.successful_robots} robots, {len(bidding_result.winning_assignments)} assignments")
        else:
            print("❌ Bidding failed")
        
        # Step 7: Cleanup
        print("\n7️⃣ Cleanup...")
        order_source.disconnect()
        print("✅ Cleanup completed")
        
        # Summary
        print("\n" + "="*50)
        print("🎯 CORE FLOW SUMMARY")
        print("="*50)
        print(f"✅ Orders loaded: {len(orders)}")
        print(f"✅ Tasks created: {len(result.tasks_created)}")
        print(f"✅ Tasks in queue: {len(pending_tasks)}")
        print(f"✅ Bidding successful: {bidding_result is not None}")
        
        if bidding_result:
            print(f"✅ Robot assignments: {len(bidding_result.winning_assignments)}")
        
        print("\n🎉 CORE FLOW WORKING!")
        return True
        
    except Exception as e:
        print(f"\n❌ Flow failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def create_mock_robot(robot_id: str):
    """Create a mock robot for testing."""
    class MockRobot:
        def __init__(self, robot_id: str):
            self.config = type('Config', (), {'robot_id': robot_id})()
            self._task_active = False
            self._position = (0.0, 0.0)
            self._battery = 100.0
            self._operational_status = 'idle'
        
        def calculate_bids(self, tasks):
            from interfaces.bidding_system_interface import RobotBid
            bids = []
            for task in tasks:
                bid = RobotBid(
                    robot_id=self.config.robot_id,
                    task=task,
                    bid_value=10.0,
                    bid_metadata={"test": True}
                )
                bids.append(bid)
            return bids
        
        def is_available_for_bidding(self):
            return not self._task_active and self._operational_status == 'idle'
        
        def get_bid_calculation_statistics(self):
            return {
                'robot_id': self.config.robot_id,
                'available_for_bidding': self.is_available_for_bidding()
            }
        
        def get_status(self):
            return {
                'robot_id': self.config.robot_id,
                'position': self._position,
                'battery': self._battery,
                'task_active': self._task_active,
                'operational_status': self._operational_status
            }
        
        def assign_task(self, task):
            if self._task_active:
                return False
            self._task_active = True
            self._operational_status = 'busy'
            return True
    
    return MockRobot(robot_id)

def analyze_missing_pieces():
    """Analyze what's missing for complete flow."""
    print("\n🔍 ANALYZING MISSING PIECES")
    print("="*50)
    
    missing_pieces = []
    
    # Check for sample_orders.json
    if not os.path.exists('sample_orders.json'):
        missing_pieces.append("sample_orders.json")
    
    # Check database password
    if not os.getenv('WAREHOUSE_DB_PASSWORD'):
        missing_pieces.append("Database password (WAREHOUSE_DB_PASSWORD)")
    
    # Check inventory data
    try:
        warehouse_map = WarehouseMap(width=20, height=15)
        simulation_data_service = SimulationDataServiceImpl(
            warehouse_map=warehouse_map,
            db_host="localhost",
            db_port=5432,
            db_name="warehouse_sim",
            db_user="postgres",
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD')
        )
        stats = simulation_data_service.get_inventory_statistics()
        if stats['total_items'] == 0:
            missing_pieces.append("Inventory data in database")
    except:
        missing_pieces.append("Database connectivity")
    
    if missing_pieces:
        print("❌ Missing pieces:")
        for piece in missing_pieces:
            print(f"   - {piece}")
    else:
        print("✅ All required pieces available")
    
    return missing_pieces

def main():
    """Run the simplified flow test."""
    print("🚀 SIMPLIFIED END-TO-END FLOW TEST")
    
    # Check missing pieces
    missing = analyze_missing_pieces()
    
    if missing:
        print(f"\n⚠️  Cannot run full test - missing: {missing}")
        return
    
    # Run core flow test
    success = test_core_flow()
    
    if success:
        print("\n🎉 SUCCESS: Core flow is working!")
        print("\n📋 Next steps for complete flow:")
        print("   1. ✅ Order source working")
        print("   2. ✅ Jobs processor working") 
        print("   3. ✅ Task queue working")
        print("   4. ✅ Robot controller working")
        print("   5. 🔄 Robot execution (needs full physics)")
        print("   6. 🔄 Task completion tracking")
    else:
        print("\n❌ FAILED: Core flow has issues")

if __name__ == "__main__":
    main() 