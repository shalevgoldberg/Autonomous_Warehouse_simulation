"""
Unit tests for Appearance Service - Visual robot appearance management.

Tests the IAppearanceService interface implementation and integration with
SharedMuJoCoEngine for Phase 1 visual shelf carrying functionality.
"""
import pytest
from unittest.mock import Mock, MagicMock

from warehouse.map import WarehouseMap
from simulation.shared_mujoco_engine import SharedMuJoCoEngine, AppearanceService
from interfaces.appearance_service_interface import IAppearanceService


@pytest.fixture
def simple_map():
    """Create a simple test map."""
    m = WarehouseMap(width=10, height=10)
    # Add a shelf for testing
    m.grid[3, 3] = 2
    return m


@pytest.fixture
def mock_engine(simple_map):
    """Create a mock SharedMuJoCoEngine for testing."""
    engine = Mock(spec=SharedMuJoCoEngine)
    engine._robot_carrying_appearance = {}
    engine.is_ready.return_value = True
    engine._model = Mock()
    engine._data = Mock()
    engine._robot_geom_ids = {"robot_1": {1, 2, 3}}

    # Mock numpy array behavior for geom_rgba
    mock_geom_rgba = Mock()
    mock_geom_rgba.__len__ = Mock(return_value=10)
    mock_geom_rgba.__setitem__ = Mock()
    engine._model.geom_rgba = mock_geom_rgba

    engine._model.nbody = 5
    return engine


@pytest.fixture
def appearance_service(mock_engine, simple_map):
    """Create AppearanceService instance for testing."""
    service = AppearanceService(mock_engine, simple_map)
    return service


class TestAppearanceService:
    """Test AppearanceService implementation."""

    def test_interface_implementation(self, appearance_service):
        """Test that AppearanceService implements IAppearanceService interface."""
        assert isinstance(appearance_service, IAppearanceService)

    def test_initial_state(self, appearance_service):
        """Test initial state of appearance service."""
        assert appearance_service.is_enabled() == True
        assert appearance_service.get_carrying_color() == (1.0, 0.5, 0.0, 1.0)

    def test_configure_service(self, appearance_service):
        """Test configuration of appearance service."""
        # Test disable service
        appearance_service.configure(enabled=False)
        assert appearance_service.is_enabled() == False

        # Test custom colors
        custom_color = (0.0, 1.0, 0.0, 1.0)
        normal_color = (0.5, 0.5, 0.5, 1.0)
        appearance_service.configure(enabled=True, carry_color=custom_color, normal_color=normal_color)

        assert appearance_service.is_enabled() == True
        assert appearance_service.get_carrying_color() == custom_color

    def test_set_carrying_appearance_disabled(self, appearance_service):
        """Test that disabled service returns False."""
        appearance_service.configure(enabled=False)

        result = appearance_service.set_carrying_appearance("robot_1", True)
        assert result == False

    def test_set_carrying_appearance_unknown_robot(self, mock_engine, simple_map):
        """Test setting appearance for unknown robot."""
        service = AppearanceService(mock_engine, simple_map)

        result = service.set_carrying_appearance("unknown_robot", True)
        assert result == False

    def test_set_carrying_appearance_success(self, mock_engine, simple_map):
        """Test successful appearance change."""
        # Setup mock engine
        mock_engine._robot_carrying_appearance = {"robot_1": False}

        service = AppearanceService(mock_engine, simple_map)

        result = service.set_carrying_appearance("robot_1", True)
        assert result == True
        assert mock_engine._robot_carrying_appearance["robot_1"] == True

    def test_set_carrying_appearance_already_set(self, mock_engine, simple_map):
        """Test setting appearance that's already in desired state."""
        # Setup mock engine with already carrying
        mock_engine._robot_carrying_appearance = {"robot_1": True}

        service = AppearanceService(mock_engine, simple_map)

        result = service.set_carrying_appearance("robot_1", True)
        assert result == True  # Should return True for already set

    def test_is_carrying_appearance(self, mock_engine, simple_map):
        """Test checking carrying appearance state."""
        mock_engine._robot_carrying_appearance = {"robot_1": True, "robot_2": False}

        service = AppearanceService(mock_engine, simple_map)

        assert service.is_carrying_appearance("robot_1") == True
        assert service.is_carrying_appearance("robot_2") == False
        assert service.is_carrying_appearance("unknown_robot") == False

    def test_engine_not_ready(self, mock_engine, simple_map):
        """Test behavior when engine is not ready."""
        mock_engine.is_ready.return_value = False
        mock_engine._robot_carrying_appearance = {"robot_1": False}

        service = AppearanceService(mock_engine, simple_map)

        result = service.set_carrying_appearance("robot_1", True)
        assert result == True  # Should return True but not apply visual change

    def test_color_application_error_handling(self, mock_engine, simple_map):
        """Test error handling in color application."""
        # Setup mock to raise exception during color application
        mock_engine._robot_carrying_appearance = {"robot_1": False}
        mock_engine._model.geom_rgba.__setitem__.side_effect = Exception("Color application failed")

        service = AppearanceService(mock_engine, simple_map)

        # Should handle exception gracefully
        result = service.set_carrying_appearance("robot_1", True)
        assert result == False


class TestAppearanceServiceIntegration:
    """Test AppearanceService integration with SharedMuJoCoEngine."""

    def test_engine_creates_appearance_service(self, simple_map):
        """Test that SharedMuJoCoEngine creates appearance service on initialization."""
        engine = SharedMuJoCoEngine(simple_map, physics_dt=0.001)

        # Initially not ready
        assert engine.get_appearance_service() is None

        # Register and initialize
        engine.register_robot("robot_1", (1.0, 1.0, 0.0))
        engine.initialize()

        # Now should have appearance service
        appearance_service = engine.get_appearance_service()
        assert appearance_service is not None
        assert isinstance(appearance_service, IAppearanceService)

    def test_robot_appearance_state_tracking(self, simple_map):
        """Test that engine tracks robot appearance state."""
        engine = SharedMuJoCoEngine(simple_map, physics_dt=0.001)

        # Register robots
        engine.register_robot("robot_1", (1.0, 1.0, 0.0))
        engine.register_robot("robot_2", (2.0, 2.0, 0.0))

        # Check initial state
        assert engine._robot_carrying_appearance["robot_1"] == False
        assert engine._robot_carrying_appearance["robot_2"] == False

        # Initialize engine
        engine.initialize()

        # Get appearance service and test state changes
        appearance_service = engine.get_appearance_service()
        assert appearance_service is not None

        # Change appearance
        appearance_service.set_carrying_appearance("robot_1", True)
        assert engine._robot_carrying_appearance["robot_1"] == True
        assert engine._robot_carrying_appearance["robot_2"] == False  # Unchanged


if __name__ == "__main__":
    pytest.main([__file__])
