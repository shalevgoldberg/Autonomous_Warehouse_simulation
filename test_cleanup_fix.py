#!/usr/bin/env python3
"""
Test the bay lock cleanup fix in RobotAgent.stop()
"""
import os
os.environ['WAREHOUSE_DB_PASSWORD'] = 'renaspolter'

from demo_robot_task_simulation import TaskSimulationManager
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap
import time

def test_cleanup_fix():
    """Test that RobotAgent.stop() properly cleans up bay locks"""
    print("🧪 Testing bay lock cleanup fix...")

    # Step 1: Clean any existing orphaned locks
    print("1️⃣ Cleaning existing orphaned locks...")
    test_service = SimulationDataServiceImpl(WarehouseMap())
    initial_orphaned = test_service.cleanup_expired_bay_locks()
    print(f"   🧹 Cleaned {initial_orphaned} existing orphaned locks")

    # Step 2: Start simulation and let it run briefly
    print("2️⃣ Starting simulation...")
    sim = TaskSimulationManager()

    try:
        sim.start_simulation()
        print("   ✅ Simulation started")

        # Let it run for a few seconds to potentially acquire locks
        print("   ⏳ Running for 5 seconds...")
        time.sleep(5)

        # Check if any locks were acquired during this time
        try:
            with test_service._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute('SELECT COUNT(*) FROM bay_locks')
                    current_locks = cur.fetchone()[0]
                    print(f"   📊 Current bay locks: {current_locks}")
        except Exception as e:
            print(f"   ⚠️  Could not check current locks: {e}")
            current_locks = 0

        # Step 3: Stop simulation (this should clean up locks)
        print("3️⃣ Stopping simulation (testing cleanup)...")
        sim.stop_simulation()
        print("   ✅ Simulation stopped")

        # Step 4: Check for orphaned locks
        print("4️⃣ Checking for orphaned locks...")
        final_orphaned = test_service.cleanup_expired_bay_locks()
        print(f"   🧹 Orphaned locks found: {final_orphaned}")

        if final_orphaned == 0:
            print("✅ SUCCESS: No orphaned locks detected!")
            return True
        else:
            print("❌ FAILURE: Orphaned locks still exist!")
            return False

    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_cleanup_fix()
    print(f"\n🎯 Test result: {'PASSED' if success else 'FAILED'}")
    exit(0 if success else 1)
