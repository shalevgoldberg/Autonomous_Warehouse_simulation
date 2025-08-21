"""
Conflict Box Integration Service Implementation

This service integrates queue and lock management services into a cohesive,
thread-safe conflict box management system following SOLID principles.
"""

import logging
import threading
import time
from typing import List, Optional, Dict, Any, Tuple
from datetime import datetime, timedelta
from contextlib import contextmanager

from interfaces.conflict_box_integration_interface import (
    IConflictBoxIntegrationService,
    ConflictBoxState,
    IntegrationMetrics
)
from interfaces.conflict_box_queue_interface import QueueStatus, LockAcquisitionResult, QueuePosition
from interfaces.conflict_box_queue_manager_interface import QueueMetrics, QueueIntegrityIssue
from interfaces.conflict_box_lock_manager_interface import LockInfo, LockHealthStatus

from warehouse.impl.conflict_box_queue_manager_impl import ConflictBoxQueueManagerImpl
from warehouse.impl.conflict_box_lock_manager_impl import ConflictBoxLockManagerImpl


class ConflictBoxIntegrationServiceImpl(IConflictBoxIntegrationService):
    """
    Implementation of the Conflict Box Integration Service.
    
    This service provides:
    - Coordinated queue and lock operations
    - Thread-safe conflict box management
    - Performance monitoring and health checks
    - Race condition prevention
    - Comprehensive error handling
    """
    
    def __init__(self, db_connection_string: str, max_connections: int = 5):
        """
        Initialize the Conflict Box Integration Service.
        
        Args:
            db_connection_string: PostgreSQL connection string
            max_connections: Maximum number of connections in the pool
        """
        self.db_connection_string = db_connection_string
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        
        # Initialize the underlying services
        self._queue_manager = ConflictBoxQueueManagerImpl(db_connection_string, max_connections)
        self._lock_manager = ConflictBoxLockManagerImpl(db_connection_string, max_connections)
        
        # Thread safety and monitoring
        self._operation_lock = threading.RLock()  # Reentrant lock for operations
        self._metrics_lock = threading.Lock()     # Separate lock for metrics updates
        self._health_check_lock = threading.Lock()  # Lock for health check scheduling
        self._shutdown_event = threading.Event()
        
        # Performance monitoring
        self._operation_metrics = {
            'total_operations': 0,
            'successful_operations': 0,
            'failed_operations': 0,
            'response_times': [],
            'concurrent_operations': 0
        }
        
        # Active operation tracking
        self._active_operations = set()
        self._active_operations_lock = threading.Lock()
        
        # Health monitoring
        self._last_health_check = datetime.now()
        self._health_check_interval = timedelta(minutes=5)
        
        self.logger.info("Conflict Box Integration Service initialized successfully")
    
    def _validate_parameters(self, box_id: str, robot_id: str, priority: int = 0) -> None:
        """Validate common parameters used across methods."""
        if not box_id or not isinstance(box_id, str):
            raise ValueError("box_id must be a non-empty string")
        if not robot_id or not isinstance(robot_id, str):
            raise ValueError("robot_id must be a non-empty string")
        if not isinstance(priority, int):
            raise ValueError("priority must be an integer")
    
    def _track_operation(self, operation_name: str):
        """Context manager for tracking operations and metrics."""
        @contextmanager
        def operation_tracker():
            start_time = time.time()
            operation_id = f"{operation_name}_{id(threading.current_thread())}_{int(start_time * 1000)}"
            
            # Track active operation
            with self._active_operations_lock:
                self._active_operations.add(operation_id)
                self._operation_metrics['concurrent_operations'] = len(self._active_operations)
            
            try:
                yield operation_id
            finally:
                # Update metrics
                end_time = time.time()
                response_time = end_time - start_time
                
                with self._metrics_lock:
                    self._operation_metrics['total_operations'] += 1
                    self._operation_metrics['response_times'].append(response_time)
                    
                    # Keep only last 1000 response times for memory management
                    if len(self._operation_metrics['response_times']) > 1000:
                        self._operation_metrics['response_times'] = \
                            self._operation_metrics['response_times'][-1000:]
                
                # Remove from active operations
                with self._active_operations_lock:
                    self._active_operations.discard(operation_id)
                    self._operation_metrics['concurrent_operations'] = len(self._active_operations)
        
        return operation_tracker()
    
    def _update_success_metrics(self, success: bool):
        """Update success/failure metrics."""
        with self._metrics_lock:
            if success:
                self._operation_metrics['successful_operations'] += 1
            else:
                self._operation_metrics['failed_operations'] += 1
    
    def request_conflict_box_access(self, box_id: str, robot_id: str, 
                                  priority: int = 0, timeout_seconds: float = 30.0) -> LockAcquisitionResult:
        """
        Request access to a conflict box with integrated queue and lock management.
        
        This method coordinates queue position management and lock acquisition
        in a thread-safe manner to prevent race conditions.
        """
        self._validate_parameters(box_id, robot_id, priority)
        
        with self._track_operation("request_access") as operation_id:
            self.logger.info(f"[{operation_id}] Robot {robot_id} requesting access to {box_id} with priority {priority}")
            
            try:
                # Use reentrant lock to prevent race conditions
                with self._operation_lock:
                    # Check if robot already has access
                    current_lock = self._lock_manager.get_lock_info(box_id)
                    if current_lock and current_lock.locked_by_robot == robot_id:
                        self.logger.info(f"[{operation_id}] Robot {robot_id} already has access to {box_id}")
                        self._update_success_metrics(True)
                        return LockAcquisitionResult(
                            success=True,
                            position=QueuePosition.LOCK_ACQUIRED,
                            estimated_wait_time=0.0,
                            queue_position=1,
                            error_message=None
                        )
                    
                    # Check if conflict box is available
                    if not current_lock:
                        # No lock exists, try to acquire immediately
                        lock_result = self._lock_manager.acquire_lock(box_id, robot_id, priority)
                        if lock_result.success:
                            self.logger.info(f"[{operation_id}] Robot {robot_id} acquired immediate access to {box_id}")
                            self._update_success_metrics(True)
                            return LockAcquisitionResult(
                                success=True,
                                position=QueuePosition.LOCK_ACQUIRED,
                                estimated_wait_time=0.0,
                                queue_position=1,
                                error_message=None
                            )
                    
                    # Conflict box is locked, add to queue
                    # Note: This is a simplified implementation - in a real system,
                    # you would implement proper queue management here
                    self.logger.info(f"[{operation_id}] Robot {robot_id} queued for {box_id}")
                    
                    # ACTUAL QUEUE IMPLEMENTATION
                    try:
                        # Add robot to queue
                        queue_position = self._queue_manager.add_robot_to_queue(box_id, robot_id, priority)
                        
                        # Calculate estimated wait time
                        estimated_wait = self._queue_manager.calculate_estimated_wait_time(box_id, robot_id)
                        
                        self.logger.info(f"[{operation_id}] Robot {robot_id} added to queue for {box_id} at position {queue_position}")
                        
                        self._update_success_metrics(True)
                        return LockAcquisitionResult(
                            success=True,
                            position=QueuePosition.QUEUED,
                            estimated_wait_time=estimated_wait,
                            queue_position=queue_position,
                            error_message=None
                        )
                        
                    except Exception as queue_error:
                        self.logger.error(f"[{operation_id}] Failed to add robot {robot_id} to queue for {box_id}: {queue_error}")
                        self._update_success_metrics(False)
                        raise RuntimeError(f"Failed to add robot to queue: {queue_error}")
                    
            except Exception as e:
                self.logger.error(f"[{operation_id}] Error requesting access for {robot_id} to {box_id}: {e}")
                self._update_success_metrics(False)
                raise RuntimeError(f"Failed to request conflict box access: {e}")
    
    def release_conflict_box_access(self, box_id: str, robot_id: str) -> bool:
        """
        Release access to a conflict box and coordinate next robot promotion.
        """
        self._validate_parameters(box_id, robot_id)
        
        with self._track_operation("release_access") as operation_id:
            self.logger.info(f"[{operation_id}] Robot {robot_id} releasing access to {box_id}")
            
            try:
                with self._operation_lock:
                    # Release the lock
                    lock_released = self._lock_manager.release_lock(box_id, robot_id)
                    if not lock_released:
                        self.logger.warning(f"[{operation_id}] Robot {robot_id} did not hold lock on {box_id}")
                        self._update_success_metrics(False)
                        return False
                    
                    # Update queue positions
                    self._queue_manager.update_queue_positions(box_id)
                    
                    # Promote next robot if available
                    next_robot = self._queue_manager.promote_next_robot(box_id)
                    if next_robot:
                        self.logger.info(f"[{operation_id}] Promoted robot {next_robot} for {box_id}")
                    
                    self.logger.info(f"[{operation_id}] Successfully released access to {box_id}")
                    self._update_success_metrics(True)
                    return True
                    
            except Exception as e:
                self.logger.error(f"[{operation_id}] Error releasing access for {robot_id} to {box_id}: {e}")
                self._update_success_metrics(False)
                raise RuntimeError(f"Failed to release conflict box access: {e}")
    
    def get_conflict_box_state(self, box_id: str) -> ConflictBoxState:
        """
        Get the complete state of a conflict box including queue and lock information.
        """
        self._validate_parameters(box_id, "dummy")  # robot_id not needed for this operation
        
        with self._track_operation("get_state") as operation_id:
            try:
                # Get queue status
                queue_status = self._queue_manager.get_queue_status(box_id)
                
                # Get lock information
                lock_info = self._lock_manager.get_lock_info(box_id)
                
                # Check health
                is_healthy = self._queue_manager.is_queue_healthy(box_id)
                
                # Performance metrics (simplified for now)
                performance_metrics = {
                    'queue_length': queue_status.queue_length,
                    'is_locked': lock_info is not None,
                    'last_operation': datetime.now().isoformat()
                }
                
                state = ConflictBoxState(
                    box_id=box_id,
                    queue_status=queue_status,
                    lock_info=lock_info,
                    is_healthy=is_healthy,
                    last_updated=datetime.now(),
                    performance_metrics=performance_metrics
                )
                
                self._update_success_metrics(True)
                return state
                
            except Exception as e:
                self.logger.error(f"[{operation_id}] Error getting state for {box_id}: {e}")
                self._update_success_metrics(False)
                raise RuntimeError(f"Failed to get conflict box state: {e}")
    
    def update_robot_heartbeat(self, box_id: str, robot_id: str) -> bool:
        """
        Update the heartbeat for a robot holding a lock on a conflict box.
        """
        self._validate_parameters(box_id, robot_id)
        
        with self._track_operation("update_heartbeat") as operation_id:
            try:
                success = self._lock_manager.update_heartbeat(box_id, robot_id)
                self._update_success_metrics(success)
                return success
                
            except Exception as e:
                self.logger.error(f"[{operation_id}] Error updating heartbeat for {robot_id} on {box_id}: {e}")
                self._update_success_metrics(False)
                raise RuntimeError(f"Failed to update robot heartbeat: {e}")
    
    def cleanup_expired_entries(self, box_id: Optional[str] = None) -> int:
        """
        Clean up expired queue entries and locks with coordinated cleanup.
        """
        if box_id is not None:
            self._validate_parameters(box_id, "dummy")
        
        with self._track_operation("cleanup") as operation_id:
            try:
                # Clean up expired queue entries
                queue_cleaned = self._queue_manager.cleanup_expired_entries(box_id)
                
                # Clean up expired locks
                locks_cleaned = self._lock_manager.cleanup_expired_locks(box_id)
                
                total_cleaned = queue_cleaned + locks_cleaned
                
                self.logger.info(f"[{operation_id}] Cleaned up {total_cleaned} expired entries "
                               f"({queue_cleaned} queue, {locks_cleaned} locks)")
                
                self._update_success_metrics(True)
                return total_cleaned
                
            except Exception as e:
                self.logger.error(f"[{operation_id}] Error during cleanup: {e}")
                self._update_success_metrics(False)
                raise RuntimeError(f"Failed to cleanup expired entries: {e}")
    
    def validate_system_integrity(self, box_id: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Validate the integrity of the entire conflict box system.
        """
        if box_id is not None:
            self._validate_parameters(box_id, "dummy")
        
        with self._track_operation("validate_integrity") as operation_id:
            try:
                issues = []
                
                # Validate queue integrity
                queue_issues = self._queue_manager.validate_queue_integrity(box_id)
                for issue in queue_issues:
                    issues.append({
                        'source': 'queue',
                        'box_id': issue.box_id,
                        'issue_type': issue.issue_type,
                        'description': issue.issue_description,
                        'severity': issue.severity
                    })
                
                # Validate lock integrity
                lock_issues = self._lock_manager.validate_locks(box_id)
                for issue in lock_issues:
                    issues.append({
                        'source': 'lock',
                        'box_id': issue['box_id'],
                        'issue_type': issue['issue_type'],
                        'description': issue['description'],
                        'severity': issue['severity']
                    })
                
                self.logger.info(f"[{operation_id}] System integrity validation completed. Found {len(issues)} issues.")
                self._update_success_metrics(True)
                return issues
                
            except Exception as e:
                self.logger.error(f"[{operation_id}] Error during system integrity validation: {e}")
                self._update_success_metrics(False)
                raise RuntimeError(f"Failed to validate system integrity: {e}")
    
    def _get_connection_pool_health(self) -> Dict[str, Any]:
        """
        Get actual connection pool health information from underlying services.
        
        Returns:
            Dictionary containing connection pool health information
        """
        try:
            pool_health = {
                'queue_manager_pool_size': 0,
                'lock_manager_pool_size': 0,
                'status': 'unknown'
            }
            
            # Get queue manager pool status
            if hasattr(self._queue_manager, '_connection_pool') and self._queue_manager._connection_pool:
                try:
                    pool_health['queue_manager_pool_size'] = self._queue_manager._connection_pool.maxconn
                    pool_health['queue_manager_pool_status'] = 'active'
                except Exception as e:
                    pool_health['queue_manager_pool_status'] = f'error: {e}'
            
            # Get lock manager pool status
            if hasattr(self._lock_manager, '_connection_pool') and self._lock_manager._connection_pool:
                try:
                    pool_health['lock_manager_pool_size'] = self._lock_manager._connection_pool.maxconn
                    pool_health['lock_manager_pool_status'] = 'active'
                except Exception as e:
                    pool_health['lock_manager_pool_status'] = f'error: {e}'
            
            # Determine overall status
            if pool_health['queue_manager_pool_size'] > 0 and pool_health['lock_manager_pool_size'] > 0:
                pool_health['status'] = 'healthy'
            elif pool_health['queue_manager_pool_size'] > 0 or pool_health['lock_manager_pool_size'] > 0:
                pool_health['status'] = 'degraded'
            else:
                pool_health['status'] = 'unhealthy'
            
            return pool_health
            
        except Exception as e:
            self.logger.error(f"Error getting connection pool health: {e}")
            return {
                'queue_manager_pool_size': 0,
                'lock_manager_pool_size': 0,
                'status': 'error',
                'error': str(e)
            }
    
    def get_system_metrics(self) -> IntegrationMetrics:
        """
        Get comprehensive metrics for the integrated conflict box system.
        """
        with self._track_operation("get_metrics") as operation_id:
            try:
                with self._metrics_lock:
                    # Calculate average response time
                    avg_response_time = 0.0
                    if self._operation_metrics['response_times']:
                        avg_response_time = sum(self._operation_metrics['response_times']) / \
                                          len(self._operation_metrics['response_times'])
                    
                    # Get connection pool health (simplified for now)
                    # ACTUAL CONNECTION POOL HEALTH MONITORING
                    connection_pool_health = self._get_connection_pool_health()
                    
                    metrics = IntegrationMetrics(
                        total_operations=self._operation_metrics['total_operations'],
                        successful_operations=self._operation_metrics['successful_operations'],
                        failed_operations=self._operation_metrics['failed_operations'],
                        average_response_time=avg_response_time,
                        concurrent_operations=self._operation_metrics['concurrent_operations'],
                        connection_pool_health=connection_pool_health,
                        last_updated=datetime.now()
                    )
                
                self._update_success_metrics(True)
                return metrics
                
            except Exception as e:
                self.logger.error(f"[{operation_id}] Error getting system metrics: {e}")
                self._update_success_metrics(False)
                raise RuntimeError(f"Failed to get system metrics: {e}")
    
    def is_system_healthy(self) -> bool:
        """
        Check if the entire conflict box system is healthy.
        """
        with self._track_operation("health_check") as operation_id:
            try:
                # Check if shutdown is requested
                if self._shutdown_event.is_set():
                    return False
                
                # THREAD-SAFE HEALTH CHECK SCHEDULING
                with self._health_check_lock:
                    now = datetime.now()
                    if now - self._last_health_check > self._health_check_interval:
                        # Perform comprehensive health check
                        issues = self.validate_system_integrity()
                        high_severity_issues = [i for i in issues if i['severity'] == 'HIGH']
                        # Always update last check to avoid repeated heavy checks under failure
                        self._last_health_check = now
                        if high_severity_issues:
                            self.logger.warning(
                                f"[{operation_id}] System health check found {len(high_severity_issues)} high-severity issues"
                            )
                            return False
                
                self._update_success_metrics(True)
                return True
                
            except Exception as e:
                self.logger.error(f"[{operation_id}] Error during health check: {e}")
                self._update_success_metrics(False)
                return False
    
    def shutdown(self) -> None:
        """
        Gracefully shutdown the integration service.
        """
        self.logger.info("Shutting down Conflict Box Integration Service...")
        
        # Set shutdown event
        self._shutdown_event.set()
        
        # Wait for active operations to complete (with timeout)
        timeout = 30  # seconds
        start_time = time.time()
        
        while self._operation_metrics['concurrent_operations'] > 0:
            if time.time() - start_time > timeout:
                self.logger.warning("Shutdown timeout reached, forcing shutdown")
                break
            time.sleep(0.1)
        
        # Shutdown underlying services
        try:
            if hasattr(self._queue_manager, 'shutdown'):
                self._queue_manager.shutdown()
            if hasattr(self._lock_manager, 'shutdown'):
                self._lock_manager.shutdown()
        except Exception as e:
            self.logger.error(f"Error during service shutdown: {e}")
        
        self.logger.info("Conflict Box Integration Service shutdown complete")
