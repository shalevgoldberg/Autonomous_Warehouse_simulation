"""
Integration tests for lane-based navigation system.

This module tests the integration between:
- PathPlannerGraphImpl and SimulationDataService
- Database operations for lanes, conflict boxes, and blocked cells
- Real database integration with PostgreSQL

Tests are designed to be run against a real database instance.
"""
import os
import time
import logging
import unittest
from unittest.mock import Mock
from typing import Optional

from interfaces.simulation_data_service_interface import (
    ISimulationDataService, 
    NavigationGraph, 
    GraphNode,
    SimulationDataServiceError
)
from interfaces.navigation_types import Point, LaneDirection, TaskType, LaneRec, BoxRec
from robot.impl.path_planner_graph_impl import PathPlannerGraphImpl
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TestLaneBasedNavigationIntegration(unittest.TestCase):
    """Test lane-based navigation integration with mocked service."""
    
    def setUp(self):
        """Set up test environment with mock service."""
        # Create mock warehouse map
        self.warehouse_map = Mock(spec=WarehouseMap)
        self.warehouse_map.width = 10
        self.warehouse_map.height = 10
        self.warehouse_map.grid_size = 1.0
        
        # Create test navigation graph
        self.test_graph = self._create_test_graph()
        
        # Mock simulation data service
        self.mock_sim_service = Mock(spec=ISimulationDataService)
        self.mock_sim_service.get_navigation_graph.return_value = self.test_graph
        self.mock_sim_service.get_blocked_cells.return_value = {}
        self.mock_sim_service.report_blocked_cell.return_value = True
        self.mock_sim_service.clear_blocked_cell.return_value = True
        
        # Create path planner with mock service
        self.path_planner = PathPlannerGraphImpl(self.mock_sim_service)
    
    def _create_test_graph(self) -> NavigationGraph:
        """Create a test navigation graph for testing."""
        # Create nodes
        nodes = {
            'node_1': GraphNode('node_1', Point(0, 0), [LaneDirection.EAST, LaneDirection.NORTH]),
            'node_2': GraphNode('node_2', Point(5, 0), [LaneDirection.WEST, LaneDirection.NORTH, LaneDirection.EAST]),
            'node_3': GraphNode('node_3', Point(10, 0), [LaneDirection.WEST, LaneDirection.NORTH]),
            'node_4': GraphNode('node_4', Point(0, 5), [LaneDirection.EAST, LaneDirection.SOUTH]),
            'node_5': GraphNode('node_5', Point(5, 5), [LaneDirection.WEST, LaneDirection.SOUTH, LaneDirection.EAST, LaneDirection.NORTH]),
            'node_6': GraphNode('node_6', Point(10, 5), [LaneDirection.WEST, LaneDirection.SOUTH]),
            'conflict_node': GraphNode('conflict_node', Point(5, 2.5), [LaneDirection.NORTH, LaneDirection.SOUTH], 
                                     is_conflict_box=True, conflict_box_id='conflict_box_1')
        }
        
        # Create edges
        edges = {
            'node_1': ['node_2', 'node_4'],
            'node_2': ['node_1', 'node_3', 'node_5', 'conflict_node'],
            'node_3': ['node_2', 'node_6'],
            'node_4': ['node_1', 'node_5'],
            'node_5': ['node_2', 'node_4', 'node_6', 'conflict_node'],
            'node_6': ['node_3', 'node_5'],
            'conflict_node': ['node_2', 'node_5']
        }
        
        # Create conflict boxes
        conflict_boxes = {
            'conflict_box_1': BoxRec(
                box_id='conflict_box_1',
                center=Point(5, 2.5),
                size=1.0
            )
        }
        
        return NavigationGraph(nodes, edges, conflict_boxes)
    
    def test_path_planning_basic(self):
        """Test basic path planning functionality."""
        start = Point(x=0.5, y=0.5)
        goal = Point(x=4.5, y=0.5)
        
        result = self.path_planner.plan_route(start, goal, TaskType.PICK_AND_DELIVER)
        
        # Verify result structure
        self.assertIsNotNone(result)
        self.assertTrue(result.success)
        self.assertIsNotNone(result.route)
        self.assertGreater(len(result.route.segments), 0)
        self.assertGreater(result.route.total_distance, 0)
        self.assertGreater(result.route.estimated_time, 0)
        
        # Verify segments have proper structure
        for segment in result.route.segments:
            self.assertIsNotNone(segment.lane)
            self.assertIsNotNone(segment.start_point)
            self.assertIsNotNone(segment.end_point)
            self.assertGreater(len(segment.lane.waypoints), 0)
        
        logger.info(f"Basic path planning test passed: {len(result.route.segments)} segments, "
                   f"{result.route.total_distance:.2f}m")
    
    def test_blocked_cells_integration(self):
        """Test blocked cells integration with path planning."""
        # Set up blocked cells
        blocked_cells = {'node_2': time.time() + 60}  # Block for 1 minute
        self.mock_sim_service.get_blocked_cells.return_value = blocked_cells
        
        start = Point(x=0.5, y=0.5)
        goal = Point(x=4.5, y=0.5)
        
        result = self.path_planner.plan_route(start, goal, TaskType.PICK_AND_DELIVER)
        
        # Verify result structure - should fail when critical node is blocked
        self.assertIsNotNone(result)
        self.assertFalse(result.success)
        self.assertIsNone(result.route)
        self.assertEqual(result.failure_reason, "path_blocked_by_obstacles")
        self.assertIsNotNone(result.error_message)
        self.assertIsNotNone(result.retry_after)
        
        # Verify blocked cells were queried
        self.mock_sim_service.get_blocked_cells.assert_called()
        
        logger.info(f"Blocked cells integration test passed: correctly detected blocked path")
    
    def test_conflict_box_awareness(self):
        """Test that conflict boxes are properly identified in routes."""
        start = Point(x=0.5, y=0.5)
        goal = Point(x=4.5, y=4.5)
        
        result = self.path_planner.plan_route(start, goal, TaskType.PICK_AND_DELIVER)
        
        # Verify result structure
        self.assertIsNotNone(result)
        self.assertTrue(result.success)
        self.assertIsNotNone(result.route)
        
        # Verify conflict boxes are identified
        self.assertIsNotNone(result.route.conflict_boxes)
        
        # If route goes through conflict area, should have conflict boxes
        if any(segment.lane.lane_id.startswith('conflict_node') or 
               segment.lane.lane_id.endswith('conflict_node') for segment in result.route.segments):
            self.assertGreater(len(result.route.conflict_boxes), 0)
        
        logger.info(f"Conflict box awareness test passed: {len(result.route.conflict_boxes)} conflict boxes identified")
    
    def test_update_block_integration(self):
        """Test updating blocked cells through path planner."""
        cell_id = 'node_2'
        unblock_time = time.time() + 30
        
        self.path_planner.update_block(cell_id, unblock_time)
        
        # Verify the service was called
        self.mock_sim_service.report_blocked_cell.assert_called_once_with(
            cell_id=cell_id,
            robot_id="path_planner",
            unblock_time=unblock_time,
            reason="path_planning_block"
        )
        
        logger.info("Update block integration test passed")
    
    def test_caching_behavior(self):
        """Test that caching works correctly for performance."""
        start = Point(x=0.5, y=0.5)
        goal = Point(x=4.5, y=0.5)
        
        # First call should load graph
        result1 = self.path_planner.plan_route(start, goal, TaskType.PICK_AND_DELIVER)
        
        # Second call should use cached graph
        result2 = self.path_planner.plan_route(start, goal, TaskType.PICK_AND_DELIVER)
        
        # Verify both results are valid
        self.assertIsNotNone(result1)
        self.assertIsNotNone(result2)
        self.assertTrue(result1.success)
        self.assertTrue(result2.success)
        self.assertIsNotNone(result1.route)
        self.assertIsNotNone(result2.route)
        
        # Graph should only be loaded once initially
        self.assertEqual(self.mock_sim_service.get_navigation_graph.call_count, 1)
        
        logger.info("Caching behavior test passed")
    
    def test_error_handling(self):
        """Test error handling in path planning."""
        # Test with service error
        self.mock_sim_service.get_navigation_graph.side_effect = SimulationDataServiceError("Database error")
        
        start = Point(x=0.5, y=0.5)
        goal = Point(x=4.5, y=0.5)
        
        result = self.path_planner.plan_route(start, goal, TaskType.PICK_AND_DELIVER)
        
        # Verify graceful failure handling
        self.assertIsNotNone(result)
        self.assertFalse(result.success)
        self.assertIsNone(result.route)
        self.assertEqual(result.failure_reason, "database_error")
        self.assertIsNotNone(result.error_message)
        self.assertIsNotNone(result.retry_after)
        
        logger.info("Error handling test passed")
    
    def test_legacy_compatibility(self):
        """Test that legacy methods still work."""
        # Reset the service to work properly
        self.mock_sim_service.get_navigation_graph.side_effect = None
        self.mock_sim_service.get_navigation_graph.return_value = self.test_graph
        
        start = (0.5, 0.5)
        goal = (4.5, 0.5)
        
        path = self.path_planner.plan_path(start, goal)
        
        # Verify legacy path format
        self.assertIsInstance(path, list)
        self.assertGreater(len(path), 0)
        
        # Each path point should be a tuple
        for point in path:
            self.assertIsInstance(point, tuple)
            self.assertEqual(len(point), 2)
        
        logger.info(f"Legacy compatibility test passed: {len(path)} path points")


class TestSimulationDataServiceLaneOperations(unittest.TestCase):
    """Test SimulationDataService lane-based operations with real database."""
    
    def setUp(self):
        """Set up test environment with real database connection."""
        # These tests require actual database connection
        if not os.getenv('WAREHOUSE_DB_PASSWORD'):
            self.skipTest("Database not available for integration tests")
        
        # Create a simple warehouse map for testing
        self.warehouse_map = WarehouseMap(width=20, height=20)
        
        # Initialize real SimulationDataService
        self.sim_service = SimulationDataServiceImpl(
            warehouse_map=self.warehouse_map,
            db_host="localhost",
            db_port=5432,
            db_name="warehouse_sim",
            db_user="postgres",
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD')
        )
        
        # Clean up any existing test data
        self._cleanup_test_data()
        
        # Insert test data
        self._insert_test_data()
    
    def tearDown(self):
        """Clean up test data after each test."""
        self._cleanup_test_data()
        if hasattr(self, 'sim_service'):
            self.sim_service.close()
    
    def _cleanup_test_data(self):
        """Clean up test data from database."""
        try:
            with self.sim_service._get_connection() as conn:
                with conn.cursor() as cur:
                    # Clean up in reverse dependency order to avoid foreign key violations
                    cur.execute("DELETE FROM blocked_cells WHERE cell_id LIKE 'test_%'")
                    cur.execute("DELETE FROM navigation_graph_edges WHERE edge_id LIKE 'test_%'")
                    cur.execute("DELETE FROM navigation_graph_nodes WHERE node_id LIKE 'test_%'")
                    cur.execute("DELETE FROM conflict_boxes WHERE box_id LIKE 'test_%'")
                    cur.execute("DELETE FROM lanes WHERE lane_id LIKE 'test_%'")
                    conn.commit()
                    logger.debug("Test data cleanup completed successfully")
        except Exception as e:
            logger.warning(f"Cleanup failed (this is normal if tables don't exist yet): {e}")
            # Don't raise the exception - cleanup failure shouldn't stop tests
    
    def _insert_test_data(self):
        """Insert test data into database."""
        try:
            with self.sim_service._get_connection() as conn:
                with conn.cursor() as cur:
                    print("Inserting test robot...")
                    cur.execute("""
                        INSERT INTO robots (robot_id, name) VALUES
                        ('test_robot_1', 'Test Robot 1')
                        ON CONFLICT (robot_id) DO NOTHING
                    """)
                    print("Inserting test lanes...")
                    cur.execute("""
                        INSERT INTO lanes (lane_id, direction, waypoints, is_goal_only) VALUES
                        ('test_lane_1', 'E', '[[0,0],[5,0]]'::jsonb, false),
                        ('test_lane_2', 'N', '[[5,0],[5,5]]'::jsonb, false),
                        ('test_lane_3', 'W', '[[5,5],[0,5]]'::jsonb, false),
                        ('test_lane_4', 'S', '[[0,5],[0,0]]'::jsonb, false)
                    """)
                    print("Inserting test conflict boxes...")
                    cur.execute("""
                        INSERT INTO conflict_boxes (box_id, center_x, center_y, size) VALUES
                        ('test_box_1', 2.5, 2.5, 1.0),
                        ('test_box_2', 7.5, 7.5, 1.5)
                    """)
                    print("Inserting test navigation graph nodes...")
                    cur.execute("""
                        INSERT INTO navigation_graph_nodes (node_id, position_x, position_y, directions, is_conflict_box, conflict_box_id) VALUES
                        ('test_node_1', 0, 0, '[\"E\",\"N\"]'::jsonb, false, null),
                        ('test_node_2', 5, 0, '[\"W\",\"N\",\"E\"]'::jsonb, false, null),
                        ('test_node_3', 5, 5, '[\"W\",\"S\",\"E\",\"N\"]'::jsonb, false, null),
                        ('test_node_4', 0, 5, '[\"E\",\"S\"]'::jsonb, false, null),
                        ('test_conflict_node', 2.5, 2.5, '[\"N\",\"S\"]'::jsonb, true, 'test_box_1')
                    """)
                    print("Inserting test navigation graph edges...")
                    cur.execute("""
                        INSERT INTO navigation_graph_edges (edge_id, from_node, to_node, direction, distance) VALUES
                        ('test_edge_1', 'test_node_1', 'test_node_2', 'E', 5.0),
                        ('test_edge_2', 'test_node_2', 'test_node_3', 'N', 5.0),
                        ('test_edge_3', 'test_node_3', 'test_node_4', 'W', 5.0),
                        ('test_edge_4', 'test_node_4', 'test_node_1', 'S', 5.0),
                        ('test_edge_5', 'test_node_1', 'test_conflict_node', 'E', 2.5),
                        ('test_edge_6', 'test_conflict_node', 'test_node_3', 'N', 2.5)
                    """)
                    conn.commit()
                    print("Test data inserted successfully!")
        except Exception as e:
            import traceback
            print(f"Failed to insert test data: {e}")
            print(traceback.format_exc())
            logger.error(f"Failed to insert test data: {e}")
            logger.error(f"Traceback: {traceback.format_exc()}")
            raise
    
    def test_lane_operations_database(self):
        """Test lane operations with real database."""
        # Test lane retrieval
        lanes = self.sim_service.get_lanes()
        
        # Should have our test lanes
        self.assertGreaterEqual(len(lanes), 4)  # At least our 4 test lanes
        
        # Find our test lanes
        test_lanes = [lane for lane in lanes if lane.lane_id.startswith('test_')]
        self.assertEqual(len(test_lanes), 4)
        
        # Verify lane properties
        east_lane = next(lane for lane in test_lanes if lane.direction == LaneDirection.EAST)
        self.assertEqual(east_lane.lane_id, 'test_lane_1')
        self.assertEqual(len(east_lane.waypoints), 2)
        self.assertEqual(east_lane.waypoints[0].x, 0)
        self.assertEqual(east_lane.waypoints[0].y, 0)
        self.assertEqual(east_lane.waypoints[1].x, 5)
        self.assertEqual(east_lane.waypoints[1].y, 0)
        
        logger.info(f"Lane operations database test passed: {len(lanes)} lanes retrieved")
    
    def test_conflict_box_operations_database(self):
        """Test conflict box operations with real database."""
        # Test conflict box retrieval
        boxes = self.sim_service.get_conflict_boxes()
        
        # Should have our test boxes
        self.assertGreaterEqual(len(boxes), 2)  # At least our 2 test boxes
        
        # Find our test boxes
        test_boxes = [box for box in boxes if box.box_id.startswith('test_')]
        self.assertEqual(len(test_boxes), 2)
        
        # Verify box properties
        box_1 = next(box for box in test_boxes if box.box_id == 'test_box_1')
        self.assertEqual(box_1.center.x, 2.5)
        self.assertEqual(box_1.center.y, 2.5)
        self.assertEqual(box_1.size, 1.0)
        
        box_2 = next(box for box in test_boxes if box.box_id == 'test_box_2')
        self.assertEqual(box_2.center.x, 7.5)
        self.assertEqual(box_2.center.y, 7.5)
        self.assertEqual(box_2.size, 1.5)
        
        logger.info(f"Conflict box operations database test passed: {len(boxes)} boxes retrieved")
    
    def test_navigation_graph_database(self):
        """Test navigation graph construction from database."""
        # Test navigation graph retrieval
        graph = self.sim_service.get_navigation_graph()
        
        # Should have our test nodes
        self.assertGreaterEqual(len(graph.nodes), 5)  # At least our 5 test nodes
        
        # Find our test nodes
        test_nodes = [node_id for node_id in graph.nodes.keys() if node_id.startswith('test_')]
        self.assertEqual(len(test_nodes), 5)
        
        # Verify node properties
        conflict_node = graph.nodes['test_conflict_node']
        self.assertTrue(conflict_node.is_conflict_box)
        self.assertEqual(conflict_node.conflict_box_id, 'test_box_1')
        self.assertEqual(conflict_node.position.x, 2.5)
        self.assertEqual(conflict_node.position.y, 2.5)
        
        # Verify edges
        total_edges = sum(len(neighbors) for neighbors in graph.edges.values())
        self.assertGreaterEqual(total_edges, 6)  # At least our 6 test edges
        
        # Test connectivity
        neighbors = graph.get_neighbors('test_node_1')
        self.assertIn('test_node_2', neighbors)
        self.assertIn('test_conflict_node', neighbors)
        
        logger.info(f"Navigation graph database test passed: {len(graph.nodes)} nodes, {total_edges} edges")
    
    def test_blocked_cells_database(self):
        """Test blocked cells operations with real database."""
        # Test initial state - no blocked cells
        blocked_cells = self.sim_service.get_blocked_cells()
        self.assertEqual(len(blocked_cells), 0)
        
        # Report a blocked cell
        current_time = time.time()
        unblock_time = current_time + 60  # Block for 1 minute
        
        success = self.sim_service.report_blocked_cell(
            cell_id='test_node_2',
            robot_id='test_robot_1',
            unblock_time=unblock_time,
            reason='test_block'
        )
        self.assertTrue(success)
        
        # Verify blocked cell is reported
        blocked_cells = self.sim_service.get_blocked_cells()
        self.assertEqual(len(blocked_cells), 1)
        self.assertIn('test_node_2', blocked_cells)
        # Compare timestamps with tolerance for precision differences
        self.assertAlmostEqual(blocked_cells['test_node_2'], unblock_time, delta=1.0)
        
        # Clear the blocked cell
        success = self.sim_service.clear_blocked_cell('test_node_2', 'test_robot_1')
        self.assertTrue(success)
        
        # Verify blocked cell is cleared
        blocked_cells = self.sim_service.get_blocked_cells()
        self.assertEqual(len(blocked_cells), 0)
        
        logger.info("Blocked cells database test passed")
    
    def test_path_planning_with_real_database(self):
        """Test path planning with real database integration."""
        # Create path planner with real service
        path_planner = PathPlannerGraphImpl(self.sim_service)
        
        # Test path planning
        start = Point(x=0.5, y=0.5)
        goal = Point(x=4.5, y=4.5)
        
        result = path_planner.plan_route(start, goal, TaskType.PICK_AND_DELIVER)
        
        # Verify successful path planning
        self.assertIsNotNone(result)
        self.assertTrue(result.success)
        self.assertIsNotNone(result.route)
        self.assertGreater(len(result.route.segments), 0)
        
        # Verify route goes through our test graph
        route_node_ids = []
        for segment in result.route.segments:
            lane_id = segment.lane.lane_id
            if '->' in lane_id:
                from_node, to_node = lane_id.split('->')
                route_node_ids.extend([from_node, to_node])
        
        # Should use some of our test nodes
        test_nodes_used = [node_id for node_id in route_node_ids if node_id.startswith('test_')]
        self.assertGreater(len(test_nodes_used), 0)
        
        logger.info(f"Path planning with real database test passed: {len(result.route.segments)} segments")


if __name__ == '__main__':
    # Run tests
    unittest.main(verbosity=2) 