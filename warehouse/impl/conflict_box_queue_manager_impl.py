"""
Conflict Box Queue Manager Implementation

This service provides high-level queue management operations for conflict boxes
in a multi-robot warehouse system. It follows SOLID principles and integrates
with the database functions implemented in Phase 1.
"""

import logging
import psycopg2
from typing import List, Optional, Dict, Any
from datetime import datetime, timedelta
from psycopg2.extras import RealDictCursor
from psycopg2.pool import SimpleConnectionPool

from interfaces.conflict_box_queue_manager_interface import (
    IConflictBoxQueueManager, 
    QueueMetrics, 
    QueueIntegrityIssue
)
from interfaces.conflict_box_queue_interface import QueueStatus


class ConflictBoxQueueManagerImpl(IConflictBoxQueueManager):
    """
    Implementation of the Conflict Box Queue Manager service.
    
    This service provides:
    - Queue position management using Phase 1 database functions
    - Robot promotion logic
    - Queue integrity validation
    - Performance monitoring
    - Cleanup operations
    """
    
    def __init__(self, db_connection_string: str, max_connections: int = 5):
        """
        Initialize the Conflict Box Queue Manager.
        
        Args:
            db_connection_string: PostgreSQL connection string
            max_connections: Maximum number of connections in the pool
        """
        self.db_connection_string = db_connection_string
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        
        # Initialize connection pool for better resource management
        try:
            self._connection_pool = SimpleConnectionPool(
                minconn=1, 
                maxconn=max_connections, 
                dsn=db_connection_string
            )
            self.logger.info(f"Connection pool initialized with max {max_connections} connections")
        except Exception as e:
            self.logger.error(f"Failed to initialize connection pool: {e}")
            self._connection_pool = None
        
        # Validate connection on initialization
        self._test_connection()
    
    def _test_connection(self) -> None:
        """Test database connection and log status without leaking pooled connections."""
        conn = None
        cursor = None
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            cursor.execute("SELECT 1")
            self.logger.info("Database connection established successfully")
        except Exception as e:
            self.logger.error(f"Failed to establish database connection: {e}")
            raise RuntimeError(f"Database connection failed: {e}")
        finally:
            try:
                if cursor:
                    cursor.close()
            finally:
                if conn:
                    self._return_connection(conn)
    
    def _get_connection(self):
        """Get a database connection from the pool with proper error handling."""
        if self._connection_pool:
            try:
                return self._connection_pool.getconn()
            except Exception as e:
                self.logger.error(f"Failed to get connection from pool: {e}")
                # Fallback to direct connection if pool fails
                return psycopg2.connect(self.db_connection_string)
        else:
            # Fallback to direct connection if no pool
            return psycopg2.connect(self.db_connection_string)
    
    def _return_connection(self, conn):
        """Return a connection to the pool or close direct connections safely."""
        if not conn:
            return
        if self._connection_pool:
            try:
                self._connection_pool.putconn(conn)
            except Exception as e:
                self.logger.error(f"Failed to return connection to pool: {e}")
                try:
                    conn.close()
                except Exception:
                    pass
        else:
            try:
                conn.close()
            except Exception:
                pass
    
    def _validate_box_id(self, box_id: str) -> None:
        """Validate box_id parameter."""
        if not box_id or not isinstance(box_id, str):
            raise ValueError("box_id must be a non-empty string")
    
    def _validate_robot_id(self, robot_id: str) -> None:
        """Validate robot_id parameter."""
        if not robot_id or not isinstance(robot_id, str):
            raise ValueError("robot_id must be a non-empty string")
    
    def _validate_priority(self, priority: int) -> None:
        """Validate priority parameter."""
        if not isinstance(priority, int):
            raise ValueError("priority must be an integer")
    
    def update_queue_positions(self, box_id: str) -> bool:
        """
        Update queue positions for a specific conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            True if positions were updated successfully, False otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If database operation fails
        """
        self._validate_box_id(box_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                # Use the Phase 1 function update_queue_positions_working
                cursor.execute(
                    "SELECT update_queue_positions_working(%s)",
                    (box_id,)
                )
                conn.commit()
                
                self.logger.info(f"Queue positions updated successfully for conflict box {box_id}")
                return True
                
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Database error updating queue positions for {box_id}: {e}")
            raise RuntimeError(f"Failed to update queue positions: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Unexpected error updating queue positions for {box_id}: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def promote_next_robot(self, box_id: str) -> Optional[str]:
        """
        Promote the next robot in line for a conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            The robot ID that was promoted, or None if no robot to promote
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If promotion operation fails
        """
        self._validate_box_id(box_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                # Use the Phase 1 function promote_next_robot_in_queue
                cursor.execute(
                    "SELECT promote_next_robot_in_queue(%s)",
                    (box_id,)
                )
                result = cursor.fetchone()
                conn.commit()
                
                promoted_robot = result[0] if result else None
                
                if promoted_robot:
                    self.logger.info(f"Robot {promoted_robot} promoted for conflict box {box_id}")
                else:
                    self.logger.info(f"No robot to promote for conflict box {box_id}")
                
                return promoted_robot
                
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Database error promoting robot for {box_id}: {e}")
            raise RuntimeError(f"Failed to promote robot: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Unexpected error promoting robot for {box_id}: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def cleanup_expired_entries(self, box_id: Optional[str] = None) -> int:
        """
        Clean up expired queue entries and locks.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            Number of entries cleaned up
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If cleanup operation fails
        """
        if box_id is not None:
            self._validate_box_id(box_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                # Use the Phase 1 function cleanup_expired_queue_entries
                if box_id:
                    cursor.execute(
                        "SELECT cleanup_expired_queue_entries(%s)",
                        (box_id,)
                    )
                else:
                    cursor.execute("SELECT cleanup_expired_queue_entries()")
                
                result = cursor.fetchone()
                conn.commit()
                
                cleaned_count = result[0] if result else 0
                
                self.logger.info(f"Cleaned up {cleaned_count} expired entries for "
                               f"{'all conflict boxes' if box_id is None else f'conflict box {box_id}'}")
                
                return cleaned_count
                
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Database error during cleanup: {e}")
            raise RuntimeError(f"Failed to cleanup expired entries: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Unexpected error during cleanup: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def validate_queue_integrity(self, box_id: Optional[str] = None) -> List[QueueIntegrityIssue]:
        """
        Validate queue integrity and report any issues.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            List of integrity issues found
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If validation operation fails
        """
        if box_id is not None:
            self._validate_box_id(box_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor(cursor_factory=RealDictCursor) as cursor:
                # Use the Phase 1 function validate_queue_integrity
                if box_id:
                    cursor.execute(
                        "SELECT * FROM validate_queue_integrity(%s)",
                        (box_id,)
                    )
                else:
                    cursor.execute("SELECT * FROM validate_queue_integrity()")
                
                results = cursor.fetchall()
                
                issues = []
                for row in results:
                    issue = QueueIntegrityIssue(
                        box_id=row['box_id'],
                        issue_type=row['issue_type'],
                        issue_description=row['issue_description'],
                        robot_count=row['robot_count'],
                        severity=row['severity']
                    )
                    issues.append(issue)
                
                self.logger.info(f"Queue integrity validation completed. Found {len(issues)} issues.")
                return issues
                
        except psycopg2.Error as e:
            self.logger.error(f"Database error during integrity validation: {e}")
            raise RuntimeError(f"Failed to validate queue integrity: {e}")
        except Exception as e:
            self.logger.error(f"Unexpected error during integrity validation: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def get_queue_metrics(self, box_id: Optional[str] = None) -> List[QueueMetrics]:
        """
        Get performance metrics for conflict box queues.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            List of queue metrics
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If metrics retrieval fails
        """
        if box_id is not None:
            self._validate_box_id(box_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor(cursor_factory=RealDictCursor) as cursor:
                # Use the new get_queue_statistics function for better performance
                if box_id:
                    cursor.execute(
                        "SELECT * FROM get_queue_statistics(%s)",
                        (box_id,)
                    )
                else:
                    cursor.execute("SELECT * FROM get_queue_statistics()")
                
                results = cursor.fetchall()
                
                metrics = []
                for row in results:
                    metric = QueueMetrics(
                        total_robots=row['total_robots'],
                        queued_robots=row['queued_robots'],
                        next_in_line_robots=row['next_in_line_robots'],
                        locked_robots=row['lock_acquired_robots'],
                        average_wait_time=float(row['average_wait_time'] or 0.0),
                        max_wait_time=float(row['max_wait_time'] or 0.0),
                        priority_distribution={}  # Will be populated separately if needed
                    )
                    metrics.append(metric)
                
                self.logger.info(f"Retrieved metrics for {len(metrics)} conflict boxes.")
                return metrics
                
        except psycopg2.Error as e:
            self.logger.error(f"Database error retrieving metrics: {e}")
            raise RuntimeError(f"Failed to retrieve queue metrics: {e}")
        except Exception as e:
            self.logger.error(f"Unexpected error retrieving metrics: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def get_queue_status(self, box_id: str) -> QueueStatus:
        """
        Get the current status of a conflict box queue.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            Current queue status
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If status retrieval fails
        """
        self._validate_box_id(box_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor(cursor_factory=RealDictCursor) as cursor:
                # Get queue entries for the conflict box
                cursor.execute("""
                    SELECT robot_id, priority, position, queue_time, timeout_time
                    FROM conflict_box_queue
                    WHERE box_id = %s
                    ORDER BY priority DESC, queue_time ASC
                """, (box_id,))
                
                queue_entries = cursor.fetchall()
                
                # Get lock information
                cursor.execute("""
                    SELECT locked_by_robot, priority, locked_at, heartbeat_at
                    FROM conflict_box_locks
                    WHERE box_id = %s
                """, (box_id,))
                
                lock_info = cursor.fetchone()
                
                # Build queue status with proper data types matching the interface
                entries = []
                for row in queue_entries:
                    entry = {
                        'robot_id': row['robot_id'],
                        'priority': row['priority'],
                        'position': row['position'],  # Now correctly string-based
                        'queue_time': row['queue_time'],
                        'timeout_time': row['timeout_time']
                    }
                    entries.append(entry)
                
                # Calculate required fields for QueueStatus interface
                current_owner = lock_info['locked_by_robot'] if lock_info else None
                next_robot = None
                if entries:
                    # Find the robot with position 'next_in_line' or first in queue
                    for entry in entries:
                        if entry['position'] == 'next_in_line':
                            next_robot = entry['robot_id']
                            break
                    if not next_robot and entries:
                        next_robot = entries[0]['robot_id']  # First in queue
                
                # Calculate average wait time
                average_wait_time = 0.0
                if entries:
                    total_wait = 0.0
                    current_time = datetime.now()
                    for entry in entries:
                        if entry['queue_time']:
                            wait_time = (current_time - entry['queue_time']).total_seconds()
                            total_wait += wait_time
                    average_wait_time = total_wait / len(entries) if entries else 0.0
                
                # Calculate lock duration stats (placeholder for now)
                lock_duration_stats = {
                    'min': 0.0,
                    'max': 0.0,
                    'avg': 0.0
                }
                
                # Create QueueStatus object matching the interface exactly
                status = QueueStatus(
                    box_id=box_id,
                    current_owner=current_owner,
                    is_locked=current_owner is not None,
                    queue_length=len(entries),
                    next_robot=next_robot,
                    average_wait_time=average_wait_time,
                    lock_duration_stats=lock_duration_stats,
                    entries=entries  # Include the entries list
                )
                
                self.logger.debug(f"Retrieved queue status for conflict box {box_id}")
                return status
                
        except psycopg2.Error as e:
            self.logger.error(f"Database error retrieving queue status for {box_id}: {e}")
            raise RuntimeError(f"Failed to retrieve queue status: {e}")
        except Exception as e:
            self.logger.error(f"Unexpected error retrieving queue status for {box_id}: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def add_robot_to_queue(self, box_id: str, robot_id: str, priority: int = 0) -> int:
        """
        Add a robot to the queue for a specific conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot identifier to add to queue
            priority: Priority level for the robot (higher = higher priority)
            
        Returns:
            The position in the queue (1-based, where 1 is next in line)
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If robot is already in queue or database operation fails
        """
        self._validate_box_id(box_id)
        self._validate_robot_id(robot_id)
        self._validate_priority(priority)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                # Check if robot is already in queue
                cursor.execute("""
                    SELECT position FROM conflict_box_queue 
                    WHERE box_id = %s AND robot_id = %s
                """, (box_id, robot_id))
                
                existing = cursor.fetchone()
                if existing:
                    raise RuntimeError(f"Robot {robot_id} is already in queue for {box_id}")
                
                # Add robot to queue
                cursor.execute("""
                    INSERT INTO conflict_box_queue (box_id, robot_id, priority, position, queue_time, timeout_time, updated_at)
                    VALUES (%s, %s, %s, 'queued', CURRENT_TIMESTAMP, 
                           CURRENT_TIMESTAMP + INTERVAL '30 seconds', CURRENT_TIMESTAMP)
                """, (box_id, robot_id, priority))
                
                # Update queue positions to maintain proper ordering
                cursor.execute("SELECT update_queue_positions_working(%s)", (box_id,))
                
                # Get the robot's position in the queue
                cursor.execute("""
                    SELECT position_rank FROM (
                        SELECT robot_id, 
                               ROW_NUMBER() OVER (
                                   PARTITION BY box_id 
                                   ORDER BY priority DESC, queue_time ASC
                               ) as position_rank
                        FROM conflict_box_queue
                        WHERE box_id = %s AND position IN ('queued', 'next_in_line')
                    ) ranked
                    WHERE robot_id = %s
                """, (box_id, robot_id))
                
                position_result = cursor.fetchone()
                if not position_result:
                    raise RuntimeError(f"Failed to determine queue position for robot {robot_id}")
                
                queue_position = position_result[0]
                
                conn.commit()
                
                self.logger.info(f"Robot {robot_id} added to queue for {box_id} at position {queue_position}")
                return queue_position
                
        except psycopg2.Error as e:
            if conn: conn.rollback()
            self.logger.error(f"Database error adding robot {robot_id} to queue for {box_id}: {e}")
            raise RuntimeError(f"Failed to add robot to queue: {e}")
        except Exception as e:
            if conn: conn.rollback()
            self.logger.error(f"Unexpected error adding robot {robot_id} to queue for {box_id}: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn: self._return_connection(conn)
    
    def remove_robot_from_queue(self, box_id: str, robot_id: str) -> bool:
        """
        Remove a robot from the queue for a specific conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot identifier to remove from queue
            
        Returns:
            True if robot was removed successfully, False otherwise
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If database operation fails
        """
        self._validate_box_id(box_id)
        self._validate_robot_id(robot_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                # Check if robot is in queue
                cursor.execute("""
                    SELECT position FROM conflict_box_queue 
                    WHERE box_id = %s AND robot_id = %s
                """, (box_id, robot_id))
                
                existing = cursor.fetchone()
                if not existing:
                    self.logger.warning(f"Robot {robot_id} not found in queue for {box_id}")
                    return False
                
                # Remove robot from queue
                cursor.execute("""
                    DELETE FROM conflict_box_queue 
                    WHERE box_id = %s AND robot_id = %s
                """, (box_id, robot_id))
                
                # Update queue positions to maintain proper ordering
                cursor.execute("SELECT update_queue_positions_working(%s)", (box_id,))
                
                conn.commit()
                
                self.logger.info(f"Robot {robot_id} removed from queue for {box_id}")
                return True
                
        except psycopg2.Error as e:
            if conn: conn.rollback()
            self.logger.error(f"Database error removing robot {robot_id} from queue for {box_id}: {e}")
            raise RuntimeError(f"Failed to remove robot from queue: {e}")
        except Exception as e:
            if conn: conn.rollback()
            self.logger.error(f"Unexpected error removing robot {robot_id} from queue for {box_id}: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn: self._return_connection(conn)
    
    def calculate_estimated_wait_time(self, box_id: str, robot_id: str) -> float:
        """
        Calculate estimated wait time for a robot in the queue.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot identifier
            
        Returns:
            Estimated wait time in seconds
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If calculation fails
        """
        self._validate_box_id(box_id)
        self._validate_robot_id(robot_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                # Fetch the robot's own queue entry
                cursor.execute(
                    """
                    SELECT position, queue_time, priority
                    FROM conflict_box_queue
                    WHERE box_id = %s AND robot_id = %s
                    """,
                    (box_id, robot_id),
                )
                row = cursor.fetchone()
                if not row:
                    return 0.0  # Robot not in queue
                position, queue_time, priority = row

                if position == 'lock_acquired':
                    return 0.0

                # Compute total waiting robots (exclude already locked)
                cursor.execute(
                    """
                    SELECT COUNT(*)
                    FROM conflict_box_queue
                    WHERE box_id = %s AND position IN ('queued','next_in_line')
                    """,
                    (box_id,),
                )
                total_waiting = cursor.fetchone()[0] or 0

                # Compute average wait time among waiting robots
                cursor.execute(
                    """
                    SELECT AVG(EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - queue_time)))
                    FROM conflict_box_queue
                    WHERE box_id = %s AND position IN ('queued','next_in_line') AND queue_time IS NOT NULL
                    """,
                    (box_id,),
                )
                avg_wait_time = cursor.fetchone()[0]

                avg_wait_float = float(avg_wait_time) if avg_wait_time else 30.0

                base_estimate = avg_wait_float * 0.5
                priority_bonus = max(0.0, float(priority) * 0.1)
                position_penalty = max(0, total_waiting - 1) * 5.0

                estimated_wait = max(0.0, base_estimate + position_penalty - priority_bonus)
                self.logger.debug(
                    f"Estimated wait time for robot {robot_id} in {box_id}: {estimated_wait:.1f}s"
                )
                return estimated_wait

        except psycopg2.Error as e:
            self.logger.error(
                f"Database error calculating wait time for robot {robot_id} in {box_id}: {e}"
            )
            return 30.0
        except Exception as e:
            self.logger.error(
                f"Unexpected error calculating wait time for robot {robot_id} in {box_id}: {e}"
            )
            return 30.0
        finally:
            if conn:
                self._return_connection(conn)
    
    def is_queue_healthy(self, box_id: str) -> bool:
        """
        Check if a conflict box queue is in a healthy state.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            True if queue is healthy, False otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If health check fails
        """
        self._validate_box_id(box_id)
        
        try:
            # Get integrity issues for this conflict box
            issues = self.validate_queue_integrity(box_id)
            
            # Queue is healthy if no high-severity issues exist
            high_severity_issues = [issue for issue in issues if issue.severity == 'HIGH']
            
            is_healthy = len(high_severity_issues) == 0
            
            self.logger.debug(f"Queue health check for {box_id}: {'HEALTHY' if is_healthy else 'UNHEALTHY'}")
            return is_healthy
            
        except Exception as e:
            self.logger.error(f"Error checking queue health for {box_id}: {e}")
            # If we can't determine health, assume unhealthy for safety
            return False
    
    def get_queue_summary(self) -> Dict[str, Any]:
        """
        Get a summary of all conflict box queues.
        
        Returns:
            Dictionary containing queue summary information
            
        Raises:
            RuntimeError: If summary retrieval fails
        """
        try:
            # Get metrics for all conflict boxes
            metrics = self.get_queue_metrics()
            
            # Get integrity issues
            issues = self.validate_queue_integrity()
            
            # Calculate summary statistics
            total_boxes = len(metrics)
            total_robots = sum(m.total_robots for m in metrics)
            total_queued = sum(m.queued_robots for m in metrics)
            total_next_in_line = sum(m.next_in_line_robots for m in metrics)
            total_locked = sum(m.locked_robots for m in metrics)
            
            # Count issues by severity
            high_issues = len([i for i in issues if i.severity == 'HIGH'])
            medium_issues = len([i for i in issues if i.severity == 'MEDIUM'])
            low_issues = len([i for i in issues if i.severity == 'LOW'])
            
            summary = {
                'total_conflict_boxes': total_boxes,
                'total_robots_in_queues': total_robots,
                'total_queued_robots': total_queued,
                'total_next_in_line_robots': total_next_in_line,
                'total_locked_robots': total_locked,
                'integrity_issues': {
                    'high': high_issues,
                    'medium': medium_issues,
                    'low': low_issues,
                    'total': len(issues)
                },
                'system_health': 'HEALTHY' if high_issues == 0 else 'UNHEALTHY',
                'timestamp': datetime.now().isoformat()
            }
            
            self.logger.info(f"Generated queue summary: {total_boxes} boxes, {total_robots} robots, {len(issues)} issues")
            return summary
            
        except Exception as e:
            self.logger.error(f"Error generating queue summary: {e}")
            raise RuntimeError(f"Failed to generate queue summary: {e}")
    
    def __del__(self):
        """Cleanup connection pool on destruction."""
        if hasattr(self, '_connection_pool') and self._connection_pool:
            try:
                self._connection_pool.closeall()
                self.logger.info("Connection pool closed")
            except Exception as e:
                self.logger.error(f"Error closing connection pool: {e}")
