"""
Core interfaces for the Autonomous Warehouse System.

This module defines all the major interfaces that components must implement
to ensure proper decoupling and testability.

Updated for lane-based navigation system redesign.
"""

# Core navigation interfaces
from .navigation_types import (
    LaneDirection, Point, LaneRec, BoxRec, RouteSegment, Route,
    TaskType, NavigationError, RoutePlanningError, LaneNotFoundError
)

# Legacy navigation interfaces (for backward compatibility)
from .path_planner_interface import (
    IPathPlanner, Cell, Path, PathPlanningError
)

# Motion and control interfaces
from .motion_executor_interface import (
    IMotionExecutor, MotionStatus, MotionCommand, MotionExecutionError
)

# Task and workflow interfaces
from .task_handler_interface import (
    ITaskHandler, Task, TaskType, TaskStatus, OperationalStatus,
    TaskHandlerStatus, TaskHandlingError
)

# Central coordination interfaces
from .central_planner_interface import (
    ICentralPlanner, RouteRequest, BlockReport, CentralPlannerError
)

from .bay_scheduler_interface import (
    IBayScheduler, BayInfo, BayRequest, BayPurpose, BaySchedulerError
)

# Other system interfaces
from .state_holder_interface import IStateHolder, RobotPhysicsState
from .coordinate_system_interface import ICoordinateSystem
from .simulation_data_service_interface import ISimulationDataService
from .jobs_processor_interface import IJobsProcessor
from .jobs_queue_interface import IJobsQueue
from .bidding_system_interface import IBiddingSystem
from .order_source_interface import IOrderSource
from .visualization_interface import IVisualization 