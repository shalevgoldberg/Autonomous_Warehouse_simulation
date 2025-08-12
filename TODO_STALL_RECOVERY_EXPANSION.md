# TODO: Stall Recovery System Expansion

## Current Status ✅
- **Implemented**: Basic retry logic with configurable attempts and delays
- **Working**: Simple stall detection and recovery with timeout handling
- **Tested**: Basic functionality with 3 retry attempts and 5-second delays

## Phase 1: Intelligent Retry Strategies 🎯

### 1.1 Stall Classification System
```python
class StallType(Enum):
    TEMPORARY_OBSTACLE = "temporary_obstacle"      # Person, moving object
    PERMANENT_BLOCKAGE = "permanent_blockage"      # Wall, fallen shelf
    TRAFFIC_JAM = "traffic_jam"                    # Multiple robots stuck
    MECHANICAL_ISSUE = "mechanical_issue"          # Hardware problem
    NAVIGATION_ERROR = "navigation_error"          # Path planning issue
    CONFLICT_BOX_TIMEOUT = "conflict_box_timeout"  # Lock acquisition failed
```

### 1.2 Adaptive Retry Delays
```python
def _calculate_retry_delay(self, stall_type: StallType, retry_count: int) -> float:
    """Calculate retry delay based on stall type and retry count."""
    base_delays = {
        StallType.TEMPORARY_OBSTACLE: 2.0,    # Quick retry for moving obstacles
        StallType.PERMANENT_BLOCKAGE: 10.0,   # Longer delay for static blocks
        StallType.TRAFFIC_JAM: 5.0,           # Medium delay for traffic
        StallType.MECHANICAL_ISSUE: 15.0,     # Long delay for hardware issues
        StallType.NAVIGATION_ERROR: 3.0,      # Quick retry for path issues
        StallType.CONFLICT_BOX_TIMEOUT: 1.0,  # Very quick for lock issues
    }
    
    base_delay = base_delays.get(stall_type, 5.0)
    exponential_backoff = base_delay * (2 ** (retry_count - 1))
    return min(exponential_backoff, 60.0)  # Cap at 60 seconds
```

### 1.3 Phase-Specific Retry Limits
```python
def _get_max_retries_for_phase(self, phase: TaskPhase) -> int:
    """Get maximum retry attempts for specific task phase."""
    phase_retry_limits = {
        TaskPhase.NAVIGATING_TO_SHELF: 5,      # More retries for navigation
        TaskPhase.NAVIGATING_TO_DROPOFF: 5,    # More retries for navigation
        TaskPhase.NAVIGATING_TO_CHARGING: 3,   # Fewer retries for charging
        TaskPhase.PICKING_ITEM: 2,             # Few retries for picking
        TaskPhase.DROPPING_ITEM: 2,            # Few retries for dropping
        TaskPhase.CHARGING_BATTERY: 1,         # No retries for charging
    }
    return phase_retry_limits.get(phase, 3)
```

## Phase 2: Path Replanning Integration 🔄

### 2.1 Automatic Path Replanning
```python
def _handle_navigation_stall(self, stall_reason: str) -> None:
    """Handle stalls during navigation with automatic replanning."""
    # Classify stall type
    stall_type = self._classify_stall(stall_reason)
    
    if stall_type in [StallType.PERMANENT_BLOCKAGE, StallType.NAVIGATION_ERROR]:
        # Trigger path replanning
        self._trigger_path_replanning()
    elif stall_type == StallType.CONFLICT_BOX_TIMEOUT:
        # Release locks and retry
        self._release_conflict_box_locks()
    else:
        # Simple retry for temporary issues
        self._simple_retry()
```

### 2.2 Alternative Route Generation
```python
def _trigger_path_replanning(self) -> None:
    """Trigger path replanning with alternative route generation."""
    current_position = self.state_holder.get_robot_state().position[:2]
    target_position = self._get_current_target_position()
    
    # Try multiple path planning strategies
    strategies = [
        self._plan_with_obstacle_avoidance,
        self._plan_with_different_start_position,
        self._plan_with_relaxed_constraints,
        self._plan_alternative_route
    ]
    
    for strategy in strategies:
        try:
            route = strategy(current_position, target_position)
            if route:
                self._switch_to_new_route(route)
                return
        except Exception as e:
            self.logger.warning(f"Path planning strategy failed: {e}")
    
    # All strategies failed
    self._handle_task_error("All path planning strategies failed")
```

### 2.3 Conflict Box Lock Management
```python
def _release_conflict_box_locks(self) -> None:
    """Release all conflict box locks and retry acquisition."""
    try:
        # Release current locks
        self.lane_follower.release_all_conflict_boxes()
        
        # Wait for other robots to clear
        time.sleep(2.0)
        
        # Retry lane following with fresh locks
        self.lane_follower.follow_route(self._planned_route, self.robot_id)
        
    except Exception as e:
        self.logger.error(f"Failed to release conflict box locks: {e}")
        self._handle_task_error("Conflict box lock management failed")
```

## Phase 3: Advanced Recovery Actions 🤖

### 3.1 Backward Movement Recovery
```python
def _attempt_backward_recovery(self) -> bool:
    """Attempt to recover by moving backward to clear blocked position."""
    try:
        # Check if backward movement is safe
        if self._is_backward_movement_safe():
            # Execute backward movement
            self.motion_executor.move_backward(distance=0.5, speed=0.2)
            time.sleep(3.0)  # Wait for movement to complete
            
            # Check if position is now clear
            if self._is_position_clear():
                return True
    except Exception as e:
        self.logger.warning(f"Backward recovery failed: {e}")
    
    return False
```

### 3.2 Sideways Movement for Narrow Passages
```python
def _attempt_sideways_recovery(self) -> bool:
    """Attempt to recover by moving sideways in narrow passages."""
    try:
        # Check available space on both sides
        left_space = self._check_side_space("left")
        right_space = self._check_side_space("right")
        
        if left_space > 0.3:  # 30cm clearance
            self.motion_executor.move_sideways(distance=left_space, direction="left")
            return True
        elif right_space > 0.3:
            self.motion_executor.move_sideways(distance=right_space, direction="right")
            return True
            
    except Exception as e:
        self.logger.warning(f"Sideways recovery failed: {e}")
    
    return False
```

### 3.3 Coordinated Recovery with Other Robots
```python
def _attempt_coordinated_recovery(self) -> bool:
    """Attempt coordinated recovery with other robots in the area."""
    try:
        # Get nearby robots
        nearby_robots = self.simulation_data_service.get_nearby_robots(
            self.robot_id, radius=5.0
        )
        
        if nearby_robots:
            # Request coordinated movement
            for robot in nearby_robots:
                if robot.robot_id != self.robot_id:
                    self._request_robot_movement(robot.robot_id, "backward")
            
            # Wait for coordination
            time.sleep(5.0)
            return True
            
    except Exception as e:
        self.logger.warning(f"Coordinated recovery failed: {e}")
    
    return False
```

## Phase 4: Monitoring and Analytics 📊

### 4.1 Stall Frequency Tracking
```python
class StallAnalytics:
    def __init__(self):
        self.stall_counts = defaultdict(int)
        self.stall_locations = defaultdict(list)
        self.recovery_success_rates = defaultdict(list)
        self.phase_stall_distribution = defaultdict(int)
    
    def record_stall(self, robot_id: str, location: Tuple[float, float], 
                    stall_type: StallType, phase: TaskPhase) -> None:
        """Record stall event for analytics."""
        self.stall_counts[robot_id] += 1
        self.stall_locations[location].append({
            'robot_id': robot_id,
            'stall_type': stall_type,
            'timestamp': time.time()
        })
        self.phase_stall_distribution[phase] += 1
    
    def record_recovery_success(self, robot_id: str, stall_type: StallType, 
                               success: bool) -> None:
        """Record recovery attempt result."""
        self.recovery_success_rates[stall_type].append(success)
```

### 4.2 Performance Metrics Integration
```python
def _log_stall_metrics(self, stall_type: StallType, recovery_success: bool) -> None:
    """Log stall and recovery metrics for performance analysis."""
    self._log_kpi_event("stall_event", {
        "robot_id": self.robot_id,
        "stall_type": stall_type.value,
        "task_phase": self._task_phase.value,
        "retry_count": self._stall_retry_count,
        "recovery_success": recovery_success,
        "stall_duration": time.time() - self._stall_start_time,
        "location": self.state_holder.get_robot_state().position[:2]
    })
```

## Phase 5: Integration with Traffic Management 🚦

### 5.1 Traffic Jam Detection and Resolution
```python
def _detect_traffic_jam(self) -> bool:
    """Detect if current stall is part of a traffic jam."""
    try:
        # Get all robots in the area
        nearby_robots = self.simulation_data_service.get_nearby_robots(
            self.robot_id, radius=10.0
        )
        
        # Count stalled robots
        stalled_robots = [r for r in nearby_robots if r.operational_status == "STALLED"]
        
        # Traffic jam if more than 2 robots are stalled nearby
        return len(stalled_robots) >= 2
        
    except Exception as e:
        self.logger.warning(f"Traffic jam detection failed: {e}")
        return False
```

### 5.2 Centralized Traffic Management
```python
def _request_traffic_assistance(self) -> None:
    """Request assistance from central traffic management system."""
    try:
        # Notify central system of traffic jam
        self.simulation_data_service.report_traffic_jam(
            robot_id=self.robot_id,
            location=self.state_holder.get_robot_state().position[:2],
            affected_robots=self._get_nearby_robot_ids()
        )
        
        # Wait for traffic management response
        self._wait_for_traffic_clearance()
        
    except Exception as e:
        self.logger.warning(f"Traffic assistance request failed: {e}")
```

## Implementation Priority 📋

### High Priority (Phase 1)
1. ✅ Basic retry logic (COMPLETED)
2. Stall classification system
3. Adaptive retry delays
4. Phase-specific retry limits

### Medium Priority (Phase 2)
1. Automatic path replanning
2. Alternative route generation
3. Conflict box lock management

### Low Priority (Phase 3-5)
1. Advanced recovery actions
2. Monitoring and analytics
3. Traffic management integration

## Testing Strategy 🧪

### Unit Tests
- Stall classification accuracy
- Retry delay calculations
- Recovery action success rates

### Integration Tests
- End-to-end stall recovery scenarios
- Path replanning with obstacles
- Multi-robot coordination

### Performance Tests
- Recovery time measurements
- Memory usage during recovery
- CPU usage during complex recovery scenarios

## Configuration Parameters ⚙️

```python
class StallRecoveryConfig:
    # Retry settings
    max_retry_attempts: int = 3
    base_retry_delay: float = 5.0
    max_retry_delay: float = 60.0
    
    # Stall classification
    temporary_obstacle_timeout: float = 10.0
    traffic_jam_detection_radius: float = 10.0
    traffic_jam_threshold: int = 2
    
    # Recovery actions
    enable_backward_recovery: bool = True
    enable_sideways_recovery: bool = True
    enable_coordinated_recovery: bool = False
    
    # Analytics
    enable_stall_analytics: bool = True
    stall_metrics_retention_days: int = 30
```

## Future Enhancements 🚀

1. **Machine Learning Integration**: Use ML to predict stall likelihood and optimize recovery strategies
2. **Predictive Maintenance**: Detect mechanical issues before they cause stalls
3. **Dynamic Route Optimization**: Continuously optimize routes based on real-time traffic conditions
4. **Human-in-the-Loop**: Allow human operators to intervene in complex stall scenarios
5. **Fleet-wide Coordination**: Coordinate recovery across the entire robot fleet 