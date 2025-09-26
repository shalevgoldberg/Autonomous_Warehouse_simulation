#!/usr/bin/env python3
"""
Test Script for End-to-End Multi-Robot Demo

This script validates that the demo can:
1. Initialize properly
2. Create external orders
3. Process orders into tasks
4. Start robots and coordination
5. Run for a short test period

Run with: python test_demo_end_to_end.py
"""
import os
import sys
import time
import tempfile
from pathlib import Path

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from demo_end_to_end_multi_robot import EndToEndMultiRobotDemo


def test_demo_initialization():
    """Test that demo initializes correctly."""
    print("🧪 Testing demo initialization...")

    try:
        demo = EndToEndMultiRobotDemo(robot_count=1)  # Single robot for faster testing
        print("✅ Demo initialization successful")
        return demo
    except Exception as e:
        print(f"❌ Demo initialization failed: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_order_creation(demo):
    """Test external order creation."""
    print("\n🧪 Testing external order creation...")

    try:
        # Create a test order
        order_id = demo.create_external_order("laptop_001", 1)
        if order_id:
            print(f"✅ External order created: {order_id}")
            return True
        else:
            print("❌ External order creation failed")
            return False
    except Exception as e:
        print(f"❌ Order creation error: {e}")
        return False


def test_order_processing(demo):
    """Test order processing into tasks."""
    print("\n🧪 Testing order processing...")

    try:
        # Refresh order source to see new orders
        demo.order_source.refresh()

        # Process orders
        tasks_created = demo.process_orders_once()
        if tasks_created > 0:
            print(f"✅ Order processed into {tasks_created} tasks")
            return True
        else:
            print("❌ No tasks created from orders")
            return False
    except Exception as e:
        print(f"❌ Order processing error: {e}")
        return False


def test_demo_startup(demo):
    """Test demo startup sequence."""
    print("\n🧪 Testing demo startup...")

    try:
        demo.start_demo()
        print("✅ Demo startup successful")
        return True
    except Exception as e:
        print(f"❌ Demo startup failed: {e}")
        return False


def test_short_demo_run(demo):
    """Test a short demo run."""
    print("\n🧪 Testing short demo run...")

    try:
        # Run for just 30 seconds
        demo.run_demo_workflow(duration_minutes=0.5)
        print("✅ Short demo run completed")
        return True
    except Exception as e:
        print(f"❌ Demo run failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("🧪 END-TO-END DEMO VALIDATION TEST")
    print("=" * 50)

    # Check database password
    if not os.getenv('WAREHOUSE_DB_PASSWORD'):
        print("❌ WAREHOUSE_DB_PASSWORD not set")
        print("   Run: $env:WAREHOUSE_DB_PASSWORD='renaspolter'")
        return

    demo = None
    try:
        # Test initialization
        demo = test_demo_initialization()
        if not demo:
            return

        # Test order creation
        if not test_order_creation(demo):
            return

        # Test order processing
        if not test_order_processing(demo):
            return

        # Test demo startup
        if not test_demo_startup(demo):
            return

        # Test short demo run
        if not test_short_demo_run(demo):
            return

        print("\n🎉 ALL TESTS PASSED!")
        print("   ✅ Demo is ready for full end-to-end execution")
        print("   🚀 Run: python demo_end_to_end_multi_robot.py")

    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test suite failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if demo:
            try:
                demo.stop_demo()
                print("✅ Demo stopped cleanly")
            except Exception as e:
                print(f"⚠️  Error stopping demo: {e}")


if __name__ == "__main__":
    main()
















