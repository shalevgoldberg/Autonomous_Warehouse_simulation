"""
Bid Calculator Implementation - scalable robot task bidding.

This implementation provides a flexible, extensible framework for calculating
robot bids on tasks. It supports multiple configurable factors and follows
SOLID principles for easy extension and modification.

Features:
- Multiple bidding factors (distance, battery, workload, etc.)
- Configurable factor weights
- Extensible factor framework
- Thread-safe operation
- Performance monitoring and statistics
- Integration with StateHolder for robot state
"""
import threading
import time
import math
import logging
from typing import List, Dict, Any, Optional, Tuple, Callable
from dataclasses import dataclass, field
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed

from interfaces.bid_calculator_interface import (
    IBidCalculator, BidFactor, BidFactorWeight, BidCalculationContext,
    BidCalculationResult, BidCalculationError
)
from interfaces.task_handler_interface import Task, TaskType
from interfaces.state_holder_interface import IStateHolder
from interfaces.bidding_system_interface import RobotBid
from interfaces.simulation_data_service_interface import ISimulationDataService


@dataclass
class FactorCalculator:
    """Represents a factor calculation function with metadata."""
    factor: BidFactor
    calculator: Callable[[BidCalculationContext, Task], float]
    description: str
    enabled: bool = True
    weight: float = 1.0


class BidCalculatorImpl(IBidCalculator):
    """
    Production implementation of bid calculator with extensible factor framework.
    
    Features:
    - Multiple configurable bidding factors
    - Thread-safe operation
    - Performance monitoring
    - Extensible architecture for new factors
    - Integration with warehouse data services
    """
    
    def __init__(self, 
                 robot_id: str,
                 simulation_data_service: Optional[ISimulationDataService] = None,
                 max_workers: int = 4):
        """
        Initialize bid calculator.
        
        Args:
            robot_id: Robot identifier
            simulation_data_service: Optional service for warehouse data
            max_workers: Maximum threads for parallel bid calculation
        """
        self.robot_id = robot_id
        self.simulation_data_service = simulation_data_service
        self.max_workers = max_workers
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Factor configuration
        self._factor_weights: Dict[BidFactor, BidFactorWeight] = {}
        self._factor_calculators: Dict[BidFactor, FactorCalculator] = {}
        
        # Statistics
        self._stats = {
            'total_calculations': 0,
            'successful_calculations': 0,
            'failed_calculations': 0,
            'average_calculation_time': 0.0,
            'total_calculation_time': 0.0,
            'last_calculation_time': None
        }
        
        # Logging
        self.logger = logging.getLogger(f"BidCalculator.{robot_id}")
        
        # Initialize default factors
        self._initialize_default_factors()
    
    def calculate_bids(self, tasks: List[Task], state_holder: IStateHolder) -> List[RobotBid]:
        """
        Calculate bids for multiple tasks in parallel.
        
        Args:
            tasks: List of tasks to calculate bids for
            state_holder: Robot state holder for current status
            
        Returns:
            List[RobotBid]: Bids for all tasks (may be empty if robot unavailable)
        """
        with self._lock:
            try:
                # Check if robot is available for bidding
                if not self.is_available_for_bidding(state_holder):
                    self.logger.debug(f"Robot {self.robot_id} not available for bidding")
                    return []
                
                # Create calculation context
                context = self._create_calculation_context(state_holder)
                
                # Calculate bids in parallel
                bids = []
                start_time = time.time()
                
                with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
                    # Submit all bid calculations
                    future_to_task = {
                        executor.submit(self._calculate_single_bid_internal, task, context): task
                        for task in tasks
                    }
                    
                    # Collect results as they complete
                    for future in as_completed(future_to_task):
                        task = future_to_task[future]
                        try:
                            bid = future.result(timeout=1.0)  # 1 second timeout per task
                            if bid:
                                bids.append(bid)
                        except Exception as e:
                            self.logger.warning(f"Failed to calculate bid for task {task.task_id}: {e}")
                            self._update_stats(False, time.time() - start_time)
                
                calculation_time = time.time() - start_time
                self._update_stats(True, calculation_time)
                
                self.logger.debug(f"Calculated {len(bids)} bids from {len(tasks)} tasks in {calculation_time:.3f}s")
                return bids
                
            except Exception as e:
                self.logger.error(f"Error calculating bids: {e}")
                raise BidCalculationError(f"Failed to calculate bids: {e}") from e
    
    def calculate_single_bid(self, task: Task, state_holder: IStateHolder) -> Optional[RobotBid]:
        """
        Calculate bid for a single task.
        
        Args:
            task: Task to calculate bid for
            state_holder: Robot state holder for current status
            
        Returns:
            Optional[RobotBid]: Bid for the task, or None if robot cannot bid
        """
        with self._lock:
            try:
                # Check if robot is available for bidding
                if not self.is_available_for_bidding(state_holder):
                    return None
                
                # Create calculation context
                context = self._create_calculation_context(state_holder)
                
                # Calculate bid
                start_time = time.time()
                bid = self._calculate_single_bid_internal(task, context)
                calculation_time = time.time() - start_time
                
                self._update_stats(bid is not None, calculation_time)
                
                return bid
                
            except Exception as e:
                self.logger.error(f"Error calculating bid for task {task.task_id}: {e}")
                self._update_stats(False, 0.0)
                return None
    
    def is_available_for_bidding(self, state_holder: IStateHolder) -> bool:
        """
        Check if robot is available for bidding.
        
        Args:
            state_holder: Robot state holder for current status
            
        Returns:
            bool: True if robot can participate in bidding
        """
        try:
            # Get robot state
            robot_state = state_holder.get_robot_state()
            
            # Check battery level
            if robot_state.battery_level < 0.2:  # 20% battery threshold
                return False
            
            # For now, we only check battery level
            # Additional checks can be added when we have more state information
            # such as operational status, current task status, etc.
            
            return True
            
        except Exception as e:
            self.logger.warning(f"Error checking robot availability: {e}")
            return False
    
    def set_bid_factor_weights(self, weights: List[BidFactorWeight]) -> None:
        """
        Set weights for different bidding factors.
        
        Args:
            weights: List of factor weights to apply
        """
        with self._lock:
            for weight_config in weights:
                self._factor_weights[weight_config.factor] = weight_config
                
                # Update calculator weight if it exists
                if weight_config.factor in self._factor_calculators:
                    self._factor_calculators[weight_config.factor].weight = weight_config.weight
                    self._factor_calculators[weight_config.factor].enabled = weight_config.enabled
            
            self.logger.info(f"Updated bid factor weights: {[w.factor.value for w in weights]}")
    
    def get_bid_factor_weights(self) -> List[BidFactorWeight]:
        """
        Get current bid factor weights.
        
        Returns:
            List[BidFactorWeight]: Current factor weight configuration
        """
        with self._lock:
            return list(self._factor_weights.values())
    
    def enable_factor(self, factor: BidFactor, enabled: bool = True) -> None:
        """
        Enable or disable a specific bidding factor.
        
        Args:
            factor: Factor to enable/disable
            enabled: True to enable, False to disable
        """
        with self._lock:
            if factor in self._factor_calculators:
                self._factor_calculators[factor].enabled = enabled
            
            if factor in self._factor_weights:
                self._factor_weights[factor].enabled = enabled
            
            self.logger.info(f"{'Enabled' if enabled else 'Disabled'} factor: {factor.value}")
    
    def get_supported_factors(self) -> List[BidFactor]:
        """
        Get list of supported bidding factors.
        
        Returns:
            List[BidFactor]: Factors supported by this calculator
        """
        with self._lock:
            return list(self._factor_calculators.keys())
    
    def get_calculation_statistics(self) -> Dict[str, Any]:
        """
        Get bid calculation statistics.
        
        Returns:
            Dict[str, Any]: Statistics including calculation times, success rates, etc.
        """
        with self._lock:
            stats = self._stats.copy()
            
            # Calculate success rate
            if stats['total_calculations'] > 0:
                stats['success_rate'] = stats['successful_calculations'] / stats['total_calculations']
            else:
                stats['success_rate'] = 0.0
            
            # Add factor information
            stats['enabled_factors'] = [
                factor.value for factor, calc in self._factor_calculators.items()
                if calc.enabled
            ]
            
            return stats
    
    # Private implementation methods
    
    def _initialize_default_factors(self) -> None:
        """Initialize default bidding factors."""
        # Distance factor
        self._factor_calculators[BidFactor.DISTANCE] = FactorCalculator(
            factor=BidFactor.DISTANCE,
            calculator=self._calculate_distance_factor,
            description="Distance from robot to task location",
            enabled=True,
            weight=0.4
        )
        
        # Battery factor
        self._factor_calculators[BidFactor.BATTERY_LEVEL] = FactorCalculator(
            factor=BidFactor.BATTERY_LEVEL,
            calculator=self._calculate_battery_factor,
            description="Robot battery level consideration",
            enabled=True,
            weight=0.3
        )
        
        # Workload factor
        self._factor_calculators[BidFactor.WORKLOAD] = FactorCalculator(
            factor=BidFactor.WORKLOAD,
            calculator=self._calculate_workload_factor,
            description="Current robot workload",
            enabled=True,
            weight=0.2
        )
        
        # Task type compatibility
        self._factor_calculators[BidFactor.TASK_TYPE_COMPATIBILITY] = FactorCalculator(
            factor=BidFactor.TASK_TYPE_COMPATIBILITY,
            calculator=self._calculate_task_type_factor,
            description="Task type compatibility with robot capabilities",
            enabled=True,
            weight=0.1
        )
        
        # Initialize default weights
        for factor, calculator in self._factor_calculators.items():
            self._factor_weights[factor] = BidFactorWeight(
                factor=factor,
                weight=calculator.weight,
                enabled=calculator.enabled
            )
    
    def _create_calculation_context(self, state_holder: IStateHolder) -> BidCalculationContext:
        """Create calculation context from robot state."""
        robot_state = state_holder.get_robot_state()
        
        return BidCalculationContext(
            robot_id=self.robot_id,
            robot_position=robot_state.position,
            battery_level=robot_state.battery_level,
            current_task=getattr(state_holder, 'current_task', None),
            operational_status=getattr(robot_state, 'operational_status', 'unknown'),
            robot_capabilities=self._get_robot_capabilities(),
            warehouse_map_info=self._get_warehouse_map_info()
        )
    
    def _calculate_single_bid_internal(self, task: Task, context: BidCalculationContext) -> Optional[RobotBid]:
        """Internal method to calculate a single bid."""
        try:
            start_time = time.time()
            
            # Calculate individual factors
            factor_values = {}
            total_weighted_value = 0.0
            total_weight = 0.0
            
            for factor, calculator in self._factor_calculators.items():
                if not calculator.enabled:
                    continue
                
                try:
                    factor_value = calculator.calculator(context, task)
                    factor_values[factor] = factor_value
                    
                    weight = self._factor_weights.get(factor, BidFactorWeight(factor, 1.0)).weight
                    total_weighted_value += factor_value * weight
                    total_weight += weight
                    
                except Exception as e:
                    self.logger.warning(f"Error calculating factor {factor.value}: {e}")
                    continue
            
            # Calculate final bid value
            if total_weight > 0:
                final_bid_value = total_weighted_value / total_weight
            else:
                final_bid_value = 1000.0  # High cost if no factors available
            
            calculation_time = time.time() - start_time
            
            # Create bid metadata
            metadata = {
                'factor_breakdown': {factor.value: value for factor, value in factor_values.items()},
                'calculation_time': calculation_time,
                'robot_position': context.robot_position,
                'battery_level': context.battery_level,
                'total_weight': total_weight
            }
            
            return RobotBid(
                robot_id=self.robot_id,
                task=task,
                bid_value=final_bid_value,
                bid_metadata=metadata
            )
            
        except Exception as e:
            self.logger.error(f"Error in bid calculation: {e}")
            return None
    
    # Factor calculation methods
    
    def _calculate_distance_factor(self, context: BidCalculationContext, task: Task) -> float:
        """Calculate distance-based factor (lower is better)."""
        robot_pos = context.robot_position[:2]  # x, y only
        
        if task.task_type == TaskType.MOVE_TO_POSITION and task.target_position:
            # Calculate distance to target position
            target_pos = task.target_position[:2]
            distance = math.sqrt((target_pos[0] - robot_pos[0])**2 + (target_pos[1] - robot_pos[1])**2)
        elif task.task_type == TaskType.PICK_AND_DELIVER and task.shelf_id:
            # Calculate distance to shelf (if shelf coordinates available)
            if self.simulation_data_service:
                try:
                    shelf_info = self.simulation_data_service.get_shelf_info(task.shelf_id)
                    if shelf_info and shelf_info.position:
                        shelf_pos = shelf_info.position[:2]
                        distance = math.sqrt((shelf_pos[0] - robot_pos[0])**2 + (shelf_pos[1] - robot_pos[1])**2)
                    else:
                        distance = 10.0  # Default distance for unknown shelf
                except Exception:
                    distance = 10.0  # Default distance if shelf lookup fails
            else:
                distance = 10.0  # Default distance without simulation data service
        else:
            distance = 5.0  # Default distance for other task types
        
        # Normalize distance (0-1 scale, lower is better)
        normalized_distance = min(distance / 20.0, 1.0)  # Cap at 20m
        return normalized_distance
    
    def _calculate_battery_factor(self, context: BidCalculationContext, task: Task) -> float:
        """Calculate battery-based factor (lower is better)."""
        battery_level = context.battery_level
        
        # Battery factor: lower battery = higher cost
        # 100% battery = 0.0 cost, 20% battery = 1.0 cost
        if battery_level <= 0.2:
            return 1.0  # Maximum cost for low battery
        else:
            # Linear interpolation: 0.2 -> 1.0, 1.0 -> 0.0
            return (1.0 - battery_level) / 0.8
    
    def _calculate_workload_factor(self, context: BidCalculationContext, task: Task) -> float:
        """Calculate workload-based factor (lower is better)."""
        # Simple workload factor based on current task
        if context.current_task:
            return 0.5  # Medium cost if robot has current task
        else:
            return 0.0  # No cost if robot is idle
    
    def _calculate_task_type_factor(self, context: BidCalculationContext, task: Task) -> float:
        """Calculate task type compatibility factor (lower is better)."""
        # Simple compatibility check
        if task.task_type == TaskType.PICK_AND_DELIVER:
            return 0.0  # Robot can handle pick and deliver
        elif task.task_type == TaskType.MOVE_TO_POSITION:
            return 0.0  # Robot can handle movement
        elif task.task_type == TaskType.MOVE_TO_CHARGING:
            return 0.1  # Slight cost for charging tasks
        else:
            return 0.5  # Higher cost for unknown task types
    
    def _get_robot_capabilities(self) -> Dict[str, Any]:
        """Get robot capabilities for factor calculation."""
        return {
            'max_speed': 1.0,  # m/s
            'max_payload': 10.0,  # kg
            'battery_capacity': 100.0,  # %
            'supported_task_types': [
                TaskType.PICK_AND_DELIVER,
                TaskType.MOVE_TO_POSITION,
                TaskType.MOVE_TO_CHARGING
            ]
        }
    
    def _get_warehouse_map_info(self) -> Optional[Dict[str, Any]]:
        """Get warehouse map information for factor calculation."""
        if self.simulation_data_service:
            try:
                return {
                    'map_width': getattr(self.simulation_data_service, 'warehouse_map', {}).get('width', 0),
                    'map_height': getattr(self.simulation_data_service, 'warehouse_map', {}).get('height', 0)
                }
            except Exception:
                return None
        return None
    
    def _update_stats(self, success: bool, calculation_time: float) -> None:
        """Update calculation statistics."""
        self._stats['total_calculations'] += 1
        if success:
            self._stats['successful_calculations'] += 1
        else:
            self._stats['failed_calculations'] += 1
        
        self._stats['total_calculation_time'] += calculation_time
        self._stats['average_calculation_time'] = (
            self._stats['total_calculation_time'] / self._stats['total_calculations']
        )
        self._stats['last_calculation_time'] = datetime.now() 