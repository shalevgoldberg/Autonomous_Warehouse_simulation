import unittest
from pathlib import Path
from interfaces.navigation_types import Point, TaskType
from robot.impl.path_planner_graph_impl import PathPlannerGraphImpl

class TestPathPlannerGraphImpl(unittest.TestCase):
    def setUp(self):
        self.planner = PathPlannerGraphImpl(Path("manual_lanes_output/lanes.csv"))

    def test_plan_route(self):
        # Pick two points near the left and right of the warehouse
        start = Point(0.75, -0.75)
        goal = Point(3.75, -5.75)
        route = self.planner.plan_route(start, goal, TaskType.PICK_AND_DELIVER)
        self.assertIsNotNone(route)
        self.assertGreater(len(route.segments), 1)
        print("Route:")
        for seg in route.segments:
            print(f"  Lane {seg.lane.lane_id} at {seg.lane.waypoints[0]}")
        print(f"Total distance: {route.total_distance:.2f} m, Estimated time: {route.estimated_time:.2f} s")

if __name__ == "__main__":
    unittest.main() 