#!/usr/bin/env python3
"""
Simple test for bidding system functionality.
This is a minimal test to isolate any hanging issues.
"""
import sys
import os

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_bidding_system_import():
    """Test that we can import the bidding system."""
    print("Testing bidding system import...")
    
    try:
        from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
        print("✅ TransparentBiddingSystem imported successfully")
        return True
    except Exception as e:
        print(f"❌ Failed to import TransparentBiddingSystem: {e}")
        return False

def test_bidding_system_creation():
    """Test that we can create a bidding system instance."""
    print("Testing bidding system creation...")
    
    try:
        from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
        bidding_system = TransparentBiddingSystem()
        print("✅ TransparentBiddingSystem created successfully")
        return True
    except Exception as e:
        print(f"❌ Failed to create TransparentBiddingSystem: {e}")
        return False

def test_bidding_system_basic_methods():
    """Test basic bidding system methods."""
    print("Testing basic bidding system methods...")
    
    try:
        from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
        from interfaces.bidding_system_interface import BiddingStrategy
        
        bidding_system = TransparentBiddingSystem()
        
        # Test basic methods
        strategy = bidding_system.get_current_strategy()
        assert strategy == BiddingStrategy.TRANSPARENT, f"Expected TRANSPARENT, got {strategy}"
        
        stats = bidding_system.get_bidding_stats()
        assert stats.total_rounds == 0, f"Expected 0 rounds, got {stats.total_rounds}"
        
        print("✅ Basic bidding system methods working")
        return True
        
    except Exception as e:
        print(f"❌ Basic methods test failed: {e}")
        return False

def test_mock_robot_creation():
    """Test creating a simple mock robot."""
    print("Testing mock robot creation...")
    
    try:
        class MockRobot:
            def __init__(self, robot_id: str):
                self.robot_id = robot_id
                self._position = (0.0, 0.0)
                self._battery = 100.0
                self._task_active = False
                self._operational_status = 'idle'
            
            def get_status(self):
                return {
                    'robot_id': self.robot_id,
                    'position': self._position,
                    'battery': self._battery,
                    'task_active': self._task_active,
                    'operational_status': self._operational_status
                }
        
        robot = MockRobot("test_robot")
        status = robot.get_status()
        assert status['robot_id'] == "test_robot"
        assert status['battery'] == 100.0
        
        print("✅ Mock robot creation successful")
        return True
        
    except Exception as e:
        print(f"❌ Mock robot test failed: {e}")
        return False

def test_bidding_system_robot_availability():
    """Test robot availability checking."""
    print("Testing robot availability checking...")
    
    try:
        from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
        
        class MockRobot:
            def __init__(self, robot_id: str):
                self.robot_id = robot_id
                self._position = (0.0, 0.0)
                self._battery = 100.0
                self._task_active = False
                self._operational_status = 'idle'
            
            def get_status(self):
                return {
                    'robot_id': self.robot_id,
                    'position': self._position,
                    'battery': self._battery,
                    'task_active': self._task_active,
                    'operational_status': self._operational_status
                }
        
        bidding_system = TransparentBiddingSystem()
        robot = MockRobot("test_robot")
        
        # Test available robot
        available = bidding_system.is_robot_available_for_task(robot, None)
        assert available == True, f"Expected True, got {available}"
        
        # Test robot with low battery
        robot._battery = 15.0
        available = bidding_system.is_robot_available_for_task(robot, None)
        assert available == False, f"Expected False for low battery, got {available}"
        
        print("✅ Robot availability checking successful")
        return True
        
    except Exception as e:
        print(f"❌ Robot availability test failed: {e}")
        return False

def main():
    """Run all tests."""
    print("=== SIMPLE BIDDING SYSTEM TESTS ===")
    print()
    
    tests = [
        test_bidding_system_import,
        test_bidding_system_creation,
        test_bidding_system_basic_methods,
        test_mock_robot_creation,
        test_bidding_system_robot_availability,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"❌ Test {test.__name__} crashed: {e}")
            failed += 1
        print()
    
    print("=== TEST RESULTS ===")
    print(f"✅ Passed: {passed}")
    print(f"❌ Failed: {failed}")
    print(f"📊 Total: {passed + failed}")
    
    if failed == 0:
        print("\n🎉 ALL TESTS PASSED! Bidding system is working correctly.")
    else:
        print(f"\n⚠️  {failed} test(s) failed. There may be issues with the bidding system.")
    
    return failed == 0

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
