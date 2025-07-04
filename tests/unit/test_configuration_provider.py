import unittest
import os
from config.configuration_provider import ConfigurationProvider
from config.configuration_sources import DefaultConfigurationSource
from interfaces.configuration_interface import ConfigurationError

class TestConfigurationProvider(unittest.TestCase):
    def setUp(self):
        # Clear any environment variables that could interfere
        for k in list(os.environ.keys()):
            if k.startswith('WAREHOUSE_'):
                del os.environ[k]

    def test_default_config_loaded(self):
        provider = ConfigurationProvider()
        robot_config = provider.get_robot_config('robot_1')
        self.assertEqual(robot_config.max_speed, 1.0)
        self.assertEqual(robot_config.lane_tolerance, 0.1)
        self.assertEqual(robot_config.conflict_box_lock_timeout, 30.0)
        self.assertEqual(provider.errors, [])

    def test_env_override(self):
        os.environ['WAREHOUSE_ROBOT_MAX_SPEED'] = '2.5'
        provider = ConfigurationProvider()
        robot_config = provider.get_robot_config('robot_1')
        self.assertEqual(robot_config.max_speed, 2.5)
        self.assertEqual(provider.errors, [])

    def test_invalid_config_validation(self):
        os.environ['WAREHOUSE_ROBOT_MAX_SPEED'] = '-1.0'  # Invalid
        provider = ConfigurationProvider()
        errors = provider.errors
        self.assertTrue(any('max_speed' in e for e in errors))

    def test_set_value_and_reload(self):
        provider = ConfigurationProvider()
        provider.set_value('robot.max_speed', 3.0)
        robot_config = provider.get_robot_config('robot_1')
        self.assertEqual(robot_config.max_speed, 3.0)
        provider.reload()
        self.assertEqual(provider.get_robot_config('robot_1').max_speed, 3.0)

    def test_get_value_metadata(self):
        provider = ConfigurationProvider()
        val = provider.get_value('robot.max_speed')
        self.assertEqual(val.value, 1.0)
        self.assertEqual(val.key, 'robot.max_speed')
        self.assertIsInstance(val.description, str)

if __name__ == '__main__':
    unittest.main() 