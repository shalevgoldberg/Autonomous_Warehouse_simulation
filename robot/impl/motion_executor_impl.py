"""
MotionExecutor Implementation - Thread-safe motion execution with atomic wheel commands.

This implementation executes motion plans by converting them to low-level wheel
commands and writing them atomically to MuJoCo simulation.
"""
import threading
import time
import math
import numpy as np
from typing import Tuple, Optional, List, Dict, Any
from dataclasses import dataclass
from enum import Enum

from interfaces.motion_executor_interface import IMotionExecutor, MotionCommand, MotionStatus, MotionExecutionError
from interfaces.path_planner_interface import Path, Cell
from interfaces.coordinate_system_interface import ICoordinateSystem
from interfaces.configuration_interface import IConfigurationProvider
from interfaces.navigation_types import Point


class MotionMode(Enum):
    """Motion execution modes."""
    STOPPED = "stopped"
    FOLLOWING_PATH = "following_path"
    SINGLE_MOVE = "single_move"
    ROTATING = "rotating"
    EMERGENCY_STOP = "emergency_stop"


@dataclass(frozen=True)
class WheelCommands:
    """Atomic wheel command structure."""
    left_wheel_velocity: float
    right_wheel_velocity: float
    timestamp: float


class MotionExecutorImpl(IMotionExecutor):
    """
    Thread-safe implementation of MotionExecutor with atomic wheel commands.
    
    Threading Model:
    - execute_*(), stop_execution(): CONTROL THREAD (called by TaskHandler)
    - update_control_loop(): CONTROL THREAD ONLY (100Hz)
    - get_*(), is_at_target(): ANY THREAD (thread-safe reads)
    - emergency_stop(): ANY THREAD (emergency use)
    
    Uses atomic wheel command writing to prevent race conditions with physics thread.
    """
    
    def __init__(self, coordinate_system: ICoordinateSystem, 
                 model=None, data=None, robot_id: str = "robot_1", physics_engine=None,
                 config_provider: Optional[IConfigurationProvider] = None):
        """
        Initialize MotionExecutor.
        
        Args:
            coordinate_system: Coordinate system for position conversions
            model: MuJoCo model (optional)
            data: MuJoCo data (optional)
            robot_id: Robot identifier
            physics_engine: Physics engine reference
            config_provider: Configuration provider for motion parameters
        """
        self.coordinate_system = coordinate_system
        self.robot_id = robot_id
        self.physics_engine = physics_engine  # Store physics engine reference
        self.config_provider = config_provider
        
        # MuJoCo integration
        self.model = model
        self.data = data
        self._left_wheel_actuator_id = None
        self._right_wheel_actuator_id = None
        
        # Motion state
        self._current_mode = MotionMode.STOPPED
        self._current_path = None
        self._current_target = None
        self._path_index = 0
        self._iteration_counter = 0  # Track control loop iterations
        self._motion_start_time = 0.0  # Initialize to 0.0 instead of None
        
        # Motion parameters - get from config provider or use defaults
        if config_provider:
            robot_config = config_provider.get_robot_config(robot_id)
            self._max_linear_velocity = robot_config.max_linear_velocity
            self._max_angular_velocity = robot_config.max_angular_velocity
            self._movement_speed = robot_config.movement_speed
            self._position_tolerance = robot_config.position_tolerance
            self._wheel_base = robot_config.wheel_base
            self._wheel_radius = robot_config.wheel_radius
        else:
            # Default values
            self._max_linear_velocity = 2.0  # m/s
            self._max_angular_velocity = 2.0  # rad/s
            self._movement_speed = 1.5  # m/s
            self._position_tolerance = 0.20  # m (20cm snap threshold for task completion)
            self._wheel_base = 0.3  # m
            self._wheel_radius = 0.05  # m
        
        # Snap-to-target parameters
        self._snap_triggered = False
        self._snap_start_time = None
        self._snap_duration = 0.5  # seconds for snap motion
        self._angular_tolerance = 0.1  # rad
        
        # Thread safety
        self._status_lock = threading.RLock()
        self._command_lock = threading.Lock()
        
        # Atomic wheel commands
        self._pending_wheel_commands = WheelCommands(
            left_wheel_velocity=0.0,
            right_wheel_velocity=0.0,
            timestamp=time.time()
        )
        
        # Status tracking
        self._motion_status = MotionStatus.IDLE
        self._reached_target_flag = False
        
        # Initialize MuJoCo IDs if model provided
        if model is not None:
            self._initialize_mujoco_ids()
    
    def _initialize_mujoco_ids(self) -> None:
        """Initialize MuJoCo actuator IDs for fast access."""
        try:
            import mujoco
            
            # Get wheel actuator IDs
            self._left_wheel_actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_motor"
            )
            self._right_wheel_actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_motor"
            )
            
        except Exception as e:
            # Fallback for testing without MuJoCo
            print(f"[MotionExecutor] Warning: Could not initialize MuJoCo IDs: {e}")
            self._left_wheel_actuator_id = 0
            self._right_wheel_actuator_id = 1
    
    def execute_path(self, path: Path) -> None:
        """
        Start executing a planned path.
        
        Args:
            path: Path to execute
            
        Raises:
            MotionExecutionError: If execution cannot be started
        """
        if not path.cells:
            raise MotionExecutionError("Path is empty")
        
        print(f"[MotionExecutor] RECEIVED path with {len(path.cells)} cells")
        print(f"[MotionExecutor] Path cells: {[(cell.x, cell.y) for cell in path.cells]}")
        
        with self._status_lock:
            # Stop any current motion
            self._stop_motion_internal()
            
            # Set new path
            # Reset snap state for new motion
            self._snap_triggered = False
            self._snap_start_time = None
            self._current_path = path
            self._path_index = 0
            self._iteration_counter = 0  # Reset iteration counter for new path
            self._current_mode = MotionMode.FOLLOWING_PATH
            self._motion_start_time = time.time()
            
            # Calculate first target
            first_cell = path.cells[0]
            self._current_target = self.coordinate_system.cell_to_world(first_cell)
            
            print(f"[MotionExecutor] Started following path with {len(path.cells)} cells")
            print(f"[MotionExecutor] First target: {self._current_target}")
    
    def execute_single_move(self, target_cell: Cell) -> None:
        """
        Execute a single move to the target cell.
        
        Args:
            target_cell: Target cell to move to
            
        Raises:
            MotionExecutionError: If move cannot be executed
        """
        if not self.coordinate_system.is_valid_cell(target_cell):
            raise MotionExecutionError(f"Invalid target cell: {target_cell}")
        
        with self._status_lock:
            # Stop any current motion
            self._stop_motion_internal()
            
            # Set single move target
            # Reset snap state for new motion
            self._snap_triggered = False
            self._snap_start_time = None
            self._current_path = None
            self._path_index = 0
            self._current_mode = MotionMode.SINGLE_MOVE
            self._current_target = self.coordinate_system.cell_to_world(target_cell)
            self._motion_start_time = time.time()
            
            print(f"[MotionExecutor] Moving to cell ({target_cell.x}, {target_cell.y})")
    
    def stop_execution(self) -> None:
        """Stop current motion execution immediately."""
        with self._status_lock:
            self._stop_motion_internal()
            print(f"[MotionExecutor] Execution stopped")
    
    def get_motion_status(self) -> MotionStatus:
        """Get current motion execution status."""
        with self._status_lock:
            if self._current_mode == MotionMode.EMERGENCY_STOP:
                return MotionStatus.ERROR
            elif self._reached_target_flag:
                # Keep the flag for a few iterations to ensure TaskHandler can read it
                # Don't clear it immediately
                return MotionStatus.REACHED_TARGET
            elif self._current_mode == MotionMode.STOPPED:
                return MotionStatus.IDLE
            elif self._current_mode in [MotionMode.FOLLOWING_PATH, MotionMode.SINGLE_MOVE, MotionMode.ROTATING]:
                return MotionStatus.EXECUTING
            else:
                return MotionStatus.IDLE
    
    def is_at_target(self, target) -> bool:
        """
        Check if robot is at the specified target position.
        
        Single Source of Truth for position accuracy - all components should
        use this method instead of implementing their own position checking.
        
        Args:
            target: Target position (Point object or tuple (x, y))
            
        Returns:
            bool: True if robot is within position tolerance of target
        """
        # Handle both Point objects and tuples
        if hasattr(target, 'x') and hasattr(target, 'y'):
            target_x, target_y = target.x, target.y
        else:
            target_x, target_y = target
        
        # Get current position from physics engine or state holder
        if hasattr(self, 'physics_engine') and self.physics_engine:
            try:
                current_pos = self.physics_engine.get_robot_pose()
                current_x, current_y = current_pos[0], current_pos[1]
            except Exception:
                # Fallback if physics engine not available
                current_x, current_y = 0.0, 0.0
        else:
            # Fallback for testing without physics
            current_x, current_y = 0.0, 0.0
        
        # Calculate distance to target
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # Use configured position tolerance as single source of truth
        return distance <= self._position_tolerance
    
    def get_current_target(self) -> Optional[Tuple[float, float]]:
        """Get the current target position being moved to."""
        with self._status_lock:
            return self._current_target
    
    def set_movement_speed(self, speed: float) -> None:
        """Set the movement speed for the robot."""
        with self._status_lock:
            self._movement_speed = max(0.0, min(speed, 2.0))  # Clamp to reasonable range
    
    def get_movement_speed(self) -> float:
        """Get current movement speed."""
        with self._status_lock:
            return self._movement_speed
    
    def emergency_stop(self) -> None:
        """Emergency stop - immediately halt all motion."""
        with self._status_lock:
            self._current_mode = MotionMode.EMERGENCY_STOP
            self._current_path = None
            self._current_target = None
            self.set_wheel_commands_atomic(0.0, 0.0)
            
            print(f"[MotionExecutor] EMERGENCY STOP")
    
    def update_control_loop(self) -> None:
        """
        Update the control loop - should be called at 100Hz.
        Handles PID control and sends commands to MuJoCo.
        
        **Atomicity**: All wheel commands must be written atomically to prevent
        race conditions with physics thread.
        """
        with self._status_lock:
            if self._current_mode == MotionMode.STOPPED or self._current_mode == MotionMode.EMERGENCY_STOP:
                return
            
            # Simulate motion control (in real implementation, would get current position from StateHolder)
            # For now, use simple simulation
            
            if self._current_mode == MotionMode.FOLLOWING_PATH:
                self._update_path_following()
            elif self._current_mode == MotionMode.SINGLE_MOVE:
                self._update_single_move()
            elif self._current_mode == MotionMode.ROTATING:
                self._update_rotation()

    def rotate_to_heading(self, target_heading_rad: float, angular_speed: Optional[float] = None) -> None:
        """Rotate in place to the requested heading."""
        with self._status_lock:
            self._stop_motion_internal()
            self._current_mode = MotionMode.ROTATING
            self._target_heading = target_heading_rad
            self._rotation_speed_cap = (
                max(0.1, min(self._max_angular_velocity, angular_speed))
                if angular_speed is not None else self._max_angular_velocity
            )
            self._motion_start_time = time.time()
            # Clear any leftover reached flag from previous segment
            self._reached_target_flag = False
            print(f"[MotionExecutor] Rotating to heading {target_heading_rad:.3f} rad (cap {self._rotation_speed_cap:.2f} rad/s)")
    
    def set_wheel_commands_atomic(self, left_vel: float, right_vel: float) -> None:
        """
        Set wheel velocities atomically to prevent race conditions.
        
        **Critical**: This method must ensure both wheel commands are written
        atomically relative to physics thread reads.
        
        Args:
            left_vel: Left wheel velocity command
            right_vel: Right wheel velocity command
        """
        with self._command_lock:
            # Create atomic wheel command
            self._pending_wheel_commands = WheelCommands(
                left_wheel_velocity=left_vel,
                right_wheel_velocity=right_vel,
                timestamp=time.time()
            )
            
            # ALWAYS try to write to physics engine first (primary method)
            if self.physics_engine is not None:
                try:
                    self.physics_engine.set_wheel_velocities(left_vel, right_vel)
                except Exception as e:
                    print(f"[MotionExecutor] Error writing to physics engine: {e}")
            
            # Fallback: Write to MuJoCo if available (legacy support)
            if self.model is not None and self.data is not None:
                self._write_to_mujoco_atomic(left_vel, right_vel)
    
    def _stop_motion_internal(self) -> None:
        """Internal: Stop motion and reset state."""
        self._current_mode = MotionMode.STOPPED
        self._current_path = None
        self._current_target = None
        self._path_index = 0
        self._iteration_counter = 0  # Reset iteration counter on stop
        # Reset snap state
        self._snap_triggered = False
        self._snap_start_time = None
        self.set_wheel_commands_atomic(0.0, 0.0)
    
    def _update_path_following(self) -> None:
        """Internal: Update path following motion."""
        if not self._current_path or self._path_index >= len(self._current_path.cells):
            # Path completed
            self._current_mode = MotionMode.STOPPED
            self._current_path = None
            self._current_target = None
            self._iteration_counter = 0  # Reset iteration counter on completion
            # Set flag to signal REACHED_TARGET
            self._reached_target_flag = True
            return
        
        # Get current position to check if we reached the waypoint
        if hasattr(self, 'physics_engine') and self.physics_engine:
            current_pos = self.physics_engine.get_robot_pose()
            current_x, current_y = current_pos[0], current_pos[1]
        else:
            current_x, current_y = 0.0, 0.0
        
        # Check if we reached current waypoint
        if self._current_target:
            target_x, target_y = self._current_target
            distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            if distance < self._position_tolerance:
                # Reached current waypoint, move to next
                self._path_index += 1
                if self._path_index < len(self._current_path.cells):
                    # Set next target
                    next_cell = self._current_path.cells[self._path_index]
                    self._current_target = self.coordinate_system.cell_to_world(next_cell)
                    # Reset snap state on waypoint advance
                    self._snap_triggered = False
                    self._snap_start_time = None
                    print(f"[MotionExecutor] Reached waypoint {self._path_index-1}, moving to waypoint {self._path_index}")
                else:
                    # Path completed
                    self._current_mode = MotionMode.STOPPED
                    self._current_path = None
                    self._current_target = None
                    self._iteration_counter = 0
                    self._reached_target_flag = True
                    print(f"[MotionExecutor] Path completed!")
                    return
        
        # Calculate motion commands toward current target
        self._calculate_motion_to_target()
    
    def _update_single_move(self) -> None:
        """Internal: Update single move motion."""
        if not self._current_target:
            self._current_mode = MotionMode.STOPPED
            return
        
        # Get current position to check if we reached the target
        if hasattr(self, 'physics_engine') and self.physics_engine:
            current_pos = self.physics_engine.get_robot_pose()
            current_x, current_y = current_pos[0], current_pos[1]
        else:
            current_x, current_y = 0.0, 0.0
        
        # Check if we should trigger snap-to-target (fix: do this before early return)
        target_x, target_y = self._current_target
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        if distance < self._position_tolerance and not self._snap_triggered:
            self._snap_triggered = True
            self._snap_start_time = time.time()
            print(f"[MotionExecutor] 🎯 Snap triggered at distance {distance:.3f}m (single move)")
        
        # Check if we reached the target
        if distance < self._position_tolerance:
            # Reached target
            self._current_mode = MotionMode.STOPPED
            self._current_target = None
            self._iteration_counter = 0
            self._reached_target_flag = True
            print(f"[MotionExecutor] Single move completed - reached target!")
            return
        
        # Calculate motion commands toward target
        self._calculate_motion_to_target()
    
    def _update_rotation(self) -> None:
        """Internal: Update rotation motion toward absolute heading."""
        # Get current pose
        if hasattr(self, 'physics_engine') and self.physics_engine:
            current_x, current_y, current_theta = self.physics_engine.get_robot_pose()
        else:
            current_theta = 0.0

        # Normalize error to [-pi, pi]
        theta_error = self._target_heading - current_theta
        while theta_error > math.pi:
            theta_error -= 2 * math.pi
        while theta_error < -math.pi:
            theta_error += 2 * math.pi

        # Stop when within tolerance
        if abs(theta_error) <= self._angular_tolerance:
            self._current_mode = MotionMode.STOPPED
            self.set_wheel_commands_atomic(0.0, 0.0)
            return

        # Proportional angular velocity with cap
        k_p = 1.2
        angular_vel = max(-self._rotation_speed_cap, min(self._rotation_speed_cap, k_p * theta_error))
        left_vel, right_vel = self._angular_to_wheel_velocities(0.0, angular_vel)
        self.set_wheel_commands_atomic(left_vel, right_vel)
    
    def _calculate_motion_to_target(self) -> None:
        """Internal: Calculate motion commands toward current target."""
        if not self._current_target:
            return
        
        # Get current position from physics (simplified - should use StateHolder)
        if hasattr(self, 'physics_engine') and self.physics_engine:
            current_pos = self.physics_engine.get_robot_pose()
            current_x, current_y, current_theta = current_pos
        else:
            # Fallback - assume we're at origin
            current_x, current_y, current_theta = 0.0, 0.0, 0.0
        
        target_x, target_y = self._current_target
        
        # Calculate direction to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = (dx**2 + dy**2)**0.5
        
        print(f"[MotionExecutor] Motion calc: pos=({current_x:.3f},{current_y:.3f},{current_theta:.3f}), "
              f"target=({target_x:.3f},{target_y:.3f}), dist={distance:.3f}")
        
        if distance < self._position_tolerance:
            # Reached target
            left_vel, right_vel = 0.0, 0.0
            print(f"[MotionExecutor] At target, stopping")
        else:
            # Calculate desired heading
            desired_theta = np.arctan2(dy, dx)
            
            # Calculate angular error
            theta_error = desired_theta - current_theta
            # Normalize angle to [-π, π]
            while theta_error > np.pi:
                theta_error -= 2 * np.pi
            while theta_error < -np.pi:
                theta_error += 2 * np.pi
            
            # Check if we should trigger snap-to-target
            if distance < self._position_tolerance and not self._snap_triggered:
                self._snap_triggered = True
                self._snap_start_time = time.time()
                print(f"[MotionExecutor] 🎯 Snap triggered at distance {distance:.3f}m")
            
            # Physics-aware snap: use high velocity for final approach
            if self._snap_triggered and self._snap_start_time is not None:
                snap_elapsed = time.time() - self._snap_start_time
                if snap_elapsed < self._snap_duration:
                    # High-velocity approach toward target
                    snap_progress = snap_elapsed / self._snap_duration
                    snap_velocity = min(3.0, distance * 10.0)  # Up to 3 m/s
                    linear_vel = snap_velocity * (1.0 - snap_progress)  # Decelerate toward end
                    angular_vel = 2.0 * theta_error  # Keep steering toward target
                    print(f"[MotionExecutor] 🚀 Snap motion: vel={linear_vel:.2f}m/s, progress={snap_progress:.1%}")
                else:
                    # Snap complete - stop motion
                    linear_vel = 0.0
                    angular_vel = 0.0
                    print(f"[MotionExecutor] ✅ Snap complete - target reached!")
            else:
                # Normal proportional control with gradual speed reduction
                linear_vel = self._movement_speed * max(0.2, 1 - abs(theta_error))  # Never less than 20% of speed
                angular_vel = 0.8 * theta_error  # Reduced from 2.0 to 0.8 for less aggressive steering
            
            # Clamp angular velocity
            angular_vel = max(-self._max_angular_velocity, min(self._max_angular_velocity, angular_vel))
            
            left_vel, right_vel = self._angular_to_wheel_velocities(linear_vel, angular_vel)
            print(f"[MotionExecutor] Calculated: linear={linear_vel:.3f}, angular={angular_vel:.3f}, "
                  f"wheels=L{left_vel:.3f}/R{right_vel:.3f}")
        
        self.set_wheel_commands_atomic(left_vel, right_vel)
    
    def _angular_to_wheel_velocities(self, linear_vel: float, angular_vel: float) -> Tuple[float, float]:
        """Convert linear and angular velocities to wheel velocities."""
        # Differential drive kinematics
        left_vel = (linear_vel - angular_vel * self._wheel_base / 2) / self._wheel_radius
        right_vel = (linear_vel + angular_vel * self._wheel_base / 2) / self._wheel_radius
        
        # Clamp to reasonable limits
        max_wheel_vel = 15.0  # rad/s (increased from 10.0 to allow higher velocities)
        left_vel = max(-max_wheel_vel, min(max_wheel_vel, left_vel))
        right_vel = max(-max_wheel_vel, min(max_wheel_vel, right_vel))
        
        return left_vel, right_vel
    
    def _write_to_mujoco_atomic(self, left_vel: float, right_vel: float) -> None:
        """Write wheel commands to physics engine."""
        print(f"[MotionExecutor] _write_to_mujoco_atomic called: L={left_vel:.3f}, R={right_vel:.3f}")
        
        # First try to write to physics engine (preferred method)
        if self.physics_engine is not None:
            try:
                print(f"[MotionExecutor] Calling physics_engine.set_wheel_velocities")
                self.physics_engine.set_wheel_velocities(left_vel, right_vel)
                print(f"[MotionExecutor] Successfully sent to physics engine")
                return  # Success - no need to try MuJoCo direct access
            except Exception as e:
                print(f"[MotionExecutor] Error writing to physics engine: {e}")
        else:
            print(f"[MotionExecutor] No physics engine available")
        
        # Fallback to direct MuJoCo access (legacy)
        if self.model is None or self.data is None:
            print(f"[MotionExecutor] No MuJoCo model/data available")
            return
        
        try:
            import mujoco
            
            # Check if actuator IDs are valid before writing
            if (self._left_wheel_actuator_id is not None and 
                self._right_wheel_actuator_id is not None and
                self._left_wheel_actuator_id >= 0 and 
                self._right_wheel_actuator_id >= 0 and
                hasattr(self.data, 'ctrl') and 
                len(self.data.ctrl) > max(self._left_wheel_actuator_id, self._right_wheel_actuator_id)):
            
            # Write commands atomically
                self.data.ctrl[self._left_wheel_actuator_id] = left_vel
                self.data.ctrl[self._right_wheel_actuator_id] = right_vel
                print(f"[MotionExecutor] Successfully sent to MuJoCo")
            else:
                # Actuators not available - this is expected for simple physics mode
                print(f"[MotionExecutor] MuJoCo actuators not available (expected)")
                
        except Exception as e:
            print(f"[MotionExecutor] Error writing to MuJoCo: {e}")
    
    # Additional methods for backward compatibility and testing
    def get_status(self) -> MotionStatus:
        """Get current motion status (alias for get_motion_status)."""
        return self.get_motion_status()
    
    def is_motion_complete(self) -> bool:
        """Check if current motion is complete."""
        return self.get_motion_status() == MotionStatus.IDLE
    
    def set_motion_parameters(self, max_linear_vel: Optional[float] = None, 
                            max_angular_vel: Optional[float] = None,
                            position_tolerance: Optional[float] = None) -> None:
        """Set motion parameters."""
        with self._status_lock:
            if max_linear_vel is not None:
                self._max_linear_velocity = max_linear_vel
            if max_angular_vel is not None:
                self._max_angular_velocity = max_angular_vel
            if position_tolerance is not None:
                self._position_tolerance = position_tolerance
    
    def force_emergency_stop(self) -> None:
        """Force emergency stop (alias for emergency_stop)."""
        self.emergency_stop()
    
    def clear_reached_target_flag(self) -> None:
        """Clear the reached target flag after TaskHandler has processed it."""
        with self._status_lock:
            self._reached_target_flag = False
    
    # Lane-based navigation methods (required by interface)
    def follow_route(self, route) -> None:
        """
        Follow a lane-based route (new interface method) using world targets.
        
        Motion layer does not reason about lanes. It receives a segment and
        drives to the segment's end point in world coordinates with the
        single-move controller.
        """
        # Defensive import for type hints without circular deps at module import
        from interfaces.navigation_types import Route
        
        if route is None or not getattr(route, 'segments', None):
            # Nothing to follow
            with self._status_lock:
                self._stop_motion_internal()
            return
        
        # Use the first (or only) segment's end point as the target.
        segment = route.segments[0]
        end_point = getattr(segment, 'end_point', None)
        if end_point is None:
            # If end point missing, fall back to start point
            end_point = getattr(segment, 'start_point', None)
        
        if end_point is None:
            # No valid target - stop
            with self._status_lock:
                self._stop_motion_internal()
            return
        
        # Configure single-move toward the world target
        with self._status_lock:
            # Stop any current motion
            self._stop_motion_internal()
            # Set world target directly (bypass grid conversion)
            self._current_path = None
            self._path_index = 0
            self._current_mode = MotionMode.SINGLE_MOVE
            self._current_target = (float(end_point.x), float(end_point.y))
            self._motion_start_time = time.time()
            self._reached_target_flag = False
            # Reset snap state for new segment
            self._snap_triggered = False
            self._snap_start_time = None
            print(f"[MotionExecutor] Following segment to world target ({end_point.x:.3f}, {end_point.y:.3f})")
    
    def set_corner_speed(self, speed: float) -> None:
        """Set corner speed for conflict boxes and bay navigation."""
        with self._status_lock:
            self._corner_speed = speed
    
    def get_corner_speed(self) -> float:
        """Get current corner speed."""
        with self._status_lock:
            return getattr(self, '_corner_speed', 0.3)  # Default corner speed

    def update_config_from_robot(self, robot_config) -> None:
        """Update motion parameters with robot-specific configuration."""
        if robot_config:
            self._max_linear_velocity = robot_config.max_linear_velocity
            self._max_angular_velocity = robot_config.max_angular_velocity
            self._movement_speed = robot_config.movement_speed
            self._position_tolerance = robot_config.position_tolerance
            self._wheel_base = robot_config.wheel_base
            self._wheel_radius = robot_config.wheel_radius 