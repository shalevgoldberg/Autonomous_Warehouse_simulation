"""
Conflict Box Lock Manager Implementation

This service provides high-level lock management operations for conflict boxes
in a multi-robot warehouse system. It follows SOLID principles and integrates
with the database functions implemented in Phase 1.
"""

import logging
import psycopg2
from typing import List, Optional, Dict, Any
from datetime import datetime, timedelta
from psycopg2.extras import RealDictCursor
from psycopg2.pool import SimpleConnectionPool

from interfaces.conflict_box_lock_manager_interface import (
    IConflictBoxLockManager, 
    LockInfo, 
    LockHealthStatus
)
from interfaces.conflict_box_queue_interface import LockAcquisitionResult
from interfaces.conflict_box_queue_interface import QueuePosition


class ConflictBoxLockManagerImpl(IConflictBoxLockManager):
    """
    Implementation of the Conflict Box Lock Manager service.
    
    This service provides:
    - Lock acquisition and release
    - Heartbeat management
    - Lock health monitoring
    - Expired lock cleanup
    - Lock validation
    """
    
    def __init__(self, db_connection_string: str, max_connections: int = 5):
        """
        Initialize the Conflict Box Lock Manager.
        
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
    
    def acquire_lock(self, box_id: str, robot_id: str, priority: int = 0) -> LockAcquisitionResult:
        """
        Attempt to acquire a lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot attempting to acquire the lock
            priority: Priority level for the lock request
            
        Returns:
            LockAcquisitionResult indicating success or failure
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If lock acquisition operation fails
        """
        self._validate_box_id(box_id)
        self._validate_robot_id(robot_id)
        self._validate_priority(priority)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                # Check if the conflict box is already locked
                cursor.execute("""
                    SELECT locked_by_robot, priority 
                    FROM conflict_box_locks 
                    WHERE box_id = %s
                """, (box_id,))
                
                existing_lock = cursor.fetchone()
                
                if existing_lock:
                    # Box is already locked
                    locked_by_robot, existing_priority = existing_lock
                    
                    if locked_by_robot == robot_id:
                        # Robot already has the lock
                        return LockAcquisitionResult(
                            success=True,
                            position=QueuePosition.LOCK_ACQUIRED,
                            estimated_wait_time=0.0,
                            queue_position=1,
                            error_message=None
                        )
                    elif priority > existing_priority:
                        # Higher priority robot can steal the lock
                        self._steal_lock(cursor, box_id, robot_id, priority)
                        conn.commit()
                        
                        self.logger.info(f"Robot {robot_id} stole lock on {box_id} from {locked_by_robot}")
                        return LockAcquisitionResult(
                            success=True,
                            position=QueuePosition.LOCK_ACQUIRED,
                            estimated_wait_time=0.0,
                            queue_position=1,
                            error_message=None
                        )
                    else:
                        # Cannot acquire lock
                        return LockAcquisitionResult(
                            success=False,
                            position=QueuePosition.QUEUED,
                            estimated_wait_time=30.0,
                            queue_position=None,
                            error_message=f"Conflict box locked by {locked_by_robot} with higher priority"
                        )
                else:
                    # Box is not locked, acquire it
                    self._create_lock(cursor, box_id, robot_id, priority)
                    conn.commit()
                    
                    self.logger.info(f"Robot {robot_id} acquired lock on {box_id}")
                    return LockAcquisitionResult(
                        success=True,
                        position=QueuePosition.LOCK_ACQUIRED,
                        estimated_wait_time=0.0,
                        queue_position=1,
                        error_message=None
                    )
                    
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Database error acquiring lock for {robot_id} on {box_id}: {e}")
            raise RuntimeError(f"Failed to acquire lock: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Unexpected error acquiring lock for {robot_id} on {box_id}: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def _create_lock(self, cursor, box_id: str, robot_id: str, priority: int) -> None:
        """Create a new lock on a conflict box."""
        cursor.execute("""
            INSERT INTO conflict_box_locks (box_id, locked_by_robot, priority, locked_at, heartbeat_at)
            VALUES (%s, %s, %s, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
        """, (box_id, robot_id, priority))
    
    def _steal_lock(self, cursor, box_id: str, robot_id: str, priority: int) -> None:
        """Steal a lock from a lower priority robot."""
        cursor.execute("""
            UPDATE conflict_box_locks 
            SET locked_by_robot = %s, priority = %s, locked_at = CURRENT_TIMESTAMP, heartbeat_at = CURRENT_TIMESTAMP
            WHERE box_id = %s
        """, (robot_id, priority, box_id))
    
    def release_lock(self, box_id: str, robot_id: str) -> bool:
        """
        Release a lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot releasing the lock
            
        Returns:
            True if lock was released successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If lock release operation fails
        """
        self._validate_box_id(box_id)
        self._validate_robot_id(robot_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                # Check if the robot actually holds the lock
                cursor.execute("""
                    SELECT locked_by_robot 
                    FROM conflict_box_locks 
                    WHERE box_id = %s
                """, (box_id,))
                
                lock_info = cursor.fetchone()
                
                if not lock_info:
                    self.logger.warning(f"No lock found on {box_id}")
                    return False
                
                if lock_info[0] != robot_id:
                    self.logger.warning(f"Robot {robot_id} does not hold lock on {box_id}")
                    return False
                
                # Release the lock
                cursor.execute("""
                    DELETE FROM conflict_box_locks 
                    WHERE box_id = %s AND locked_by_robot = %s
                """, (box_id, robot_id))
                
                conn.commit()
                
                self.logger.info(f"Robot {robot_id} released lock on {box_id}")
                return True
                
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Database error releasing lock for {robot_id} on {box_id}: {e}")
            raise RuntimeError(f"Failed to release lock: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Unexpected error releasing lock for {robot_id} on {box_id}: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def update_heartbeat(self, box_id: str, robot_id: str) -> bool:
        """
        Update the heartbeat for a lock to indicate the robot is still active.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot holding the lock
            
        Returns:
            True if heartbeat was updated successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If heartbeat update fails
        """
        self._validate_box_id(box_id)
        self._validate_robot_id(robot_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                # Update the heartbeat timestamp
                cursor.execute("""
                    UPDATE conflict_box_locks 
                    SET heartbeat_at = CURRENT_TIMESTAMP
                    WHERE box_id = %s AND locked_by_robot = %s
                """, (box_id, robot_id))
                
                rows_updated = cursor.rowcount
                conn.commit()
                
                if rows_updated > 0:
                    self.logger.debug(f"Updated heartbeat for robot {robot_id} on {box_id}")
                    return True
                else:
                    self.logger.warning(f"No lock found for robot {robot_id} on {box_id}")
                    return False
                    
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Database error updating heartbeat for {robot_id} on {box_id}: {e}")
            raise RuntimeError(f"Failed to update heartbeat: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Unexpected error updating heartbeat for {robot_id} on {box_id}: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def get_lock_info(self, box_id: str) -> Optional[LockInfo]:
        """
        Get information about the current lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            LockInfo if a lock exists, None otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If lock info retrieval fails
        """
        self._validate_box_id(box_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor(cursor_factory=RealDictCursor) as cursor:
                cursor.execute("""
                    SELECT box_id, locked_by_robot, priority, locked_at, heartbeat_at, 
                           lock_timeout_seconds, robot_inside
                    FROM conflict_box_locks 
                    WHERE box_id = %s
                """, (box_id,))
                
                lock_data = cursor.fetchone()
                
                if lock_data:
                    lock_info = LockInfo(
                        box_id=lock_data['box_id'],
                        locked_by_robot=lock_data['locked_by_robot'],
                        priority=lock_data['priority'],
                        locked_at=lock_data['locked_at'],
                        heartbeat_at=lock_data['heartbeat_at'],
                        lock_timeout_seconds=lock_data['lock_timeout_seconds'] or 30,
                        robot_inside=lock_data['robot_inside'] or False
                    )
                    return lock_info
                else:
                    return None
                    
        except psycopg2.Error as e:
            self.logger.error(f"Database error retrieving lock info for {box_id}: {e}")
            raise RuntimeError(f"Failed to retrieve lock info: {e}")
        except Exception as e:
            self.logger.error(f"Unexpected error retrieving lock info for {box_id}: {e}")
            raise RuntimeError(f"Unexpected error: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def is_locked(self, box_id: str) -> bool:
        """
        Check if a conflict box is currently locked.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            True if the box is locked, False otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If lock check fails
        """
        self._validate_box_id(box_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                cursor.execute("""
                    SELECT 1 FROM conflict_box_locks WHERE box_id = %s
                """, (box_id,))
                
                return cursor.fetchone() is not None
                
        except psycopg2.Error as e:
            self.logger.error(f"Database error checking lock status for {box_id}: {e}")
            raise RuntimeError(f"Failed to check lock status: {e}")
        except Exception as e:
            self.logger.error(f"Unexpected error checking lock status for {box_id}: {e}")
            raise RuntimeError(f"Failed to check lock status: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def get_locked_by_robot(self, box_id: str) -> Optional[str]:
        """
        Get the robot ID that currently holds the lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            Robot ID if locked, None otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If robot ID retrieval fails
        """
        self._validate_box_id(box_id)
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                cursor.execute("""
                    SELECT locked_by_robot FROM conflict_box_locks WHERE box_id = %s
                """, (box_id,))
                
                result = cursor.fetchone()
                return result[0] if result else None
                
        except psycopg2.Error as e:
            self.logger.error(f"Database error retrieving robot ID for {box_id}: {e}")
            raise RuntimeError(f"Failed to retrieve robot ID: {e}")
        except Exception as e:
            self.logger.error(f"Unexpected error retrieving robot ID for {box_id}: {e}")
            raise RuntimeError(f"Failed to retrieve robot ID: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def check_lock_health(self, box_id: str) -> Optional[LockHealthStatus]:
        """
        Check the health status of a lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            LockHealthStatus if a lock exists, None otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If health check fails
        """
        self._validate_box_id(box_id)
        
        try:
            lock_info = self.get_lock_info(box_id)
            if not lock_info:
                return None
            
            # Calculate health metrics
            now = datetime.now()
            time_since_heartbeat = now - lock_info.heartbeat_at
            is_expired = time_since_heartbeat > timedelta(seconds=lock_info.lock_timeout_seconds)
            is_healthy = not is_expired
            
            health_status = LockHealthStatus(
                box_id=box_id,
                is_healthy=is_healthy,
                last_heartbeat=lock_info.heartbeat_at,
                time_since_heartbeat=time_since_heartbeat,
                is_expired=is_expired,
                robot_id=lock_info.locked_by_robot
            )
            
            return health_status
            
        except Exception as e:
            self.logger.error(f"Error checking lock health for {box_id}: {e}")
            raise RuntimeError(f"Failed to check lock health: {e}")
    
    def cleanup_expired_locks(self, box_id: Optional[str] = None) -> int:
        """
        Clean up expired locks based on heartbeat timeout.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            Number of expired locks cleaned up
            
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
                # Clean up expired locks (heartbeat older than 30 seconds)
                if box_id:
                    cursor.execute("""
                        DELETE FROM conflict_box_locks 
                        WHERE box_id = %s AND heartbeat_at < CURRENT_TIMESTAMP - INTERVAL '30 seconds'
                    """, (box_id,))
                else:
                    cursor.execute("""
                        DELETE FROM conflict_box_locks 
                        WHERE heartbeat_at < CURRENT_TIMESTAMP - INTERVAL '30 seconds'
                    """)
                
                expired_count = cursor.rowcount
                conn.commit()
                
                self.logger.info(f"Cleaned up {expired_count} expired locks for "
                               f"{'all conflict boxes' if box_id is None else f'conflict box {box_id}'}")
                
                return expired_count
                
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Database error during lock cleanup: {e}")
            raise RuntimeError(f"Failed to cleanup expired locks: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Unexpected error during lock cleanup: {e}")
            raise RuntimeError(f"Failed to cleanup expired locks: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def validate_locks(self, box_id: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Validate all locks and report any issues.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            List of lock validation issues found
            
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
                # Check for expired locks
                if box_id:
                    cursor.execute("""
                        SELECT box_id, locked_by_robot, heartbeat_at,
                               EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_at)) as seconds_since_heartbeat
                        FROM conflict_box_locks 
                        WHERE box_id = %s AND heartbeat_at < CURRENT_TIMESTAMP - INTERVAL '30 seconds'
                    """, (box_id,))
                else:
                    cursor.execute("""
                        SELECT box_id, locked_by_robot, heartbeat_at,
                               EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_at)) as seconds_since_heartbeat
                        FROM conflict_box_locks 
                        WHERE heartbeat_at < CURRENT_TIMESTAMP - INTERVAL '30 seconds'
                    """)
                
                expired_locks = cursor.fetchall()
                
                # Check for locks without corresponding queue entries
                if box_id:
                    cursor.execute("""
                        SELECT cbl.box_id, cbl.locked_by_robot
                        FROM conflict_box_locks cbl
                        LEFT JOIN conflict_box_queue cbq ON cbl.box_id = cbq.box_id AND cbl.locked_by_robot = cbq.robot_id
                        WHERE cbl.box_id = %s AND cbq.robot_id IS NULL
                    """, (box_id,))
                else:
                    cursor.execute("""
                        SELECT cbl.box_id, cbl.locked_by_robot
                        FROM conflict_box_locks cbl
                        LEFT JOIN conflict_box_queue cbq ON cbl.box_id = cbq.box_id AND cbl.locked_by_robot = cbq.robot_id
                        WHERE cbq.robot_id IS NULL
                    """)
                
                orphaned_locks = cursor.fetchall()
                
                # Build validation issues
                issues = []
                
                for lock in expired_locks:
                    issues.append({
                        'box_id': lock['box_id'],
                        'issue_type': 'expired_lock',
                        'description': f"Lock expired {lock['seconds_since_heartbeat']:.1f} seconds ago",
                        'severity': 'HIGH',
                        'robot_id': lock['locked_by_robot']
                    })
                
                for lock in orphaned_locks:
                    issues.append({
                        'box_id': lock['box_id'],
                        'issue_type': 'orphaned_lock',
                        'description': 'Lock exists but robot not in queue',
                        'severity': 'MEDIUM',
                        'robot_id': lock['locked_by_robot']
                    })
                
                self.logger.info(f"Lock validation completed. Found {len(issues)} issues.")
                return issues
                
        except psycopg2.Error as e:
            self.logger.error(f"Database error during lock validation: {e}")
            raise RuntimeError(f"Failed to validate locks: {e}")
        except Exception as e:
            self.logger.error(f"Unexpected error during lock validation: {e}")
            raise RuntimeError(f"Failed to validate locks: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def get_locks_summary(self) -> Dict[str, Any]:
        """
        Get a summary of all conflict box locks.
        
        Returns:
            Dictionary containing lock summary information
            
        Raises:
            RuntimeError: If summary retrieval fails
        """
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor(cursor_factory=RealDictCursor) as cursor:
                # Get total locks count
                cursor.execute("SELECT COUNT(*) as total_locks FROM conflict_box_locks")
                total_locks = cursor.fetchone()['total_locks']
                
                # Get expired locks count
                cursor.execute("""
                    SELECT COUNT(*) as expired_locks 
                    FROM conflict_box_locks 
                    WHERE heartbeat_at < CURRENT_TIMESTAMP - INTERVAL '30 seconds'
                """)
                expired_locks = cursor.fetchone()['expired_locks']
                
                # Get locks by priority
                cursor.execute("""
                    SELECT priority, COUNT(*) as count 
                    FROM conflict_box_locks 
                    GROUP BY priority 
                    ORDER BY priority DESC
                """)
                priority_distribution = {row['priority']: row['count'] for row in cursor.fetchall()}
                
                summary = {
                    'total_locks': total_locks,
                    'expired_locks': expired_locks,
                    'healthy_locks': total_locks - expired_locks,
                    'priority_distribution': priority_distribution,
                    'system_health': 'HEALTHY' if expired_locks == 0 else 'UNHEALTHY',
                    'timestamp': datetime.now().isoformat()
                }
                
                self.logger.info(f"Generated locks summary: {total_locks} total, {expired_locks} expired")
                return summary
                
        except Exception as e:
            self.logger.error(f"Error generating locks summary: {e}")
            raise RuntimeError(f"Failed to generate locks summary: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def set_robot_inside_status(self, box_id: str, robot_id: str, is_inside: bool) -> bool:
        """
        Set whether a robot is inside a conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot identifier
            is_inside: Whether the robot is inside the box
            
        Returns:
            True if status was updated successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If status update fails
        """
        self._validate_box_id(box_id)
        self._validate_robot_id(robot_id)
        if not isinstance(is_inside, bool):
            raise ValueError("is_inside must be a boolean")
        
        conn = None
        try:
            conn = self._get_connection()
            with conn.cursor() as cursor:
                cursor.execute("""
                    UPDATE conflict_box_locks 
                    SET robot_inside = %s
                    WHERE box_id = %s AND locked_by_robot = %s
                """, (is_inside, box_id, robot_id))
                
                rows_updated = cursor.rowcount
                conn.commit()
                
                if rows_updated > 0:
                    self.logger.info(f"Updated robot inside status for {robot_id} on {box_id}: {is_inside}")
                    return True
                else:
                    self.logger.warning(f"No lock found for robot {robot_id} on {box_id}")
                    return False
                    
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Database error updating robot inside status for {robot_id} on {box_id}: {e}")
            raise RuntimeError(f"Failed to update robot inside status: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            self.logger.error(f"Unexpected error updating robot inside status for {robot_id} on {box_id}: {e}")
            raise RuntimeError(f"Failed to update robot inside status: {e}")
        finally:
            if conn:
                self._return_connection(conn)
    
    def __del__(self):
        """Cleanup connection pool on destruction."""
        if hasattr(self, '_connection_pool') and self._connection_pool:
            try:
                self._connection_pool.closeall()
                self.logger.info("Connection pool closed")
            except Exception as e:
                self.logger.error(f"Error closing connection pool: {e}")
