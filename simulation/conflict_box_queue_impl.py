"""
Conflict Box Queue Implementation - Database-backed queue management system.

This implementation provides thread-safe, priority-based queue management for conflict
box locks with comprehensive timeout handling, deadlock prevention, and statistics.

Design Principles:
- **Thread-Safe**: All operations use database transactions for consistency
- **SOLID**: Single responsibility, dependency injection, interface segregation
- **Scalable**: Efficient database operations with proper indexing
- **Robust**: Comprehensive error handling and recovery mechanisms
- **Observable**: Detailed logging and statistics for monitoring
"""
import os
import logging
import threading
import time
import uuid
from contextlib import contextmanager
from typing import Dict, Any, Optional, List, Tuple, Callable
from dataclasses import dataclass
import heapq

import psycopg2
import psycopg2.pool
from psycopg2.extras import RealDictCursor
import psycopg2.errorcodes as errorcodes

from interfaces.conflict_box_queue_interface import (
    IConflictBoxQueue,
    IConflictBoxQueueNotifier,
    QueueEntry,
    QueueStatus,
    LockAcquisitionResult,
    QueuePosition,
    ConflictBoxQueueError
)


# Configure logging
logger = logging.getLogger(__name__)


@dataclass
class PriorityQueueEntry:
    """Priority queue entry for efficient sorting."""
    priority: int  # Higher = more important
    queue_time: float  # Unix timestamp (for tie-breaking)
    robot_id: str
    box_id: str
    timeout_time: float
    
    def __lt__(self, other):
        # Higher priority first, then earlier queue time
        if self.priority != other.priority:
            return self.priority > other.priority
        return self.queue_time < other.queue_time


class ConflictBoxQueueImpl(IConflictBoxQueue):
    """
    Database-backed implementation of conflict box queue management.
    
    Features:
    - Priority-based queue ordering
    - Timeout handling with automatic cleanup
    - Comprehensive statistics and monitoring
    - Thread-safe operations with database consistency
    - Deadlock prevention through timeout mechanisms
    """
    
    def __init__(self, 
                 db_host: str = "localhost",
                 db_port: int = 5432,
                 db_name: str = "warehouse_sim",
                 db_user: str = "postgres",
                 db_password: Optional[str] = None,
                 pool_size: int = 10,
                 default_timeout: float = 30.0,
                 heartbeat_interval: float = 5.0,
                 cleanup_interval: float = 10.0):
        """
        Initialize the conflict box queue system.
        
        Args:
            db_host: Database host
            db_port: Database port
            db_name: Database name
            db_user: Database user
            db_password: Database password (or from WAREHOUSE_DB_PASSWORD env var)
            pool_size: Connection pool size
            default_timeout: Default timeout for queue requests (seconds)
            heartbeat_interval: Heartbeat interval for active locks (seconds)
            cleanup_interval: Cleanup interval for expired entries (seconds)
        """
        # Get database password from environment if not provided
        if db_password is None:
            db_password = os.getenv('WAREHOUSE_DB_PASSWORD')
            if not db_password:
                raise ConflictBoxQueueError(
                    "Database password not provided. Set WAREHOUSE_DB_PASSWORD environment variable."
                )
        
        # Database connection parameters
        self.db_params = {
            'host': db_host,
            'port': db_port,
            'database': db_name,
            'user': db_user,
            'password': db_password,
            'cursor_factory': RealDictCursor,
            'connect_timeout': 10,
            'application_name': 'conflict_box_queue'
        }
        
        # Configuration
        self.default_timeout = default_timeout
        self.heartbeat_interval = heartbeat_interval
        self.cleanup_interval = cleanup_interval
        
        # Connection pool for thread safety
        self._connection_pool = None
        self._pool_lock = threading.Lock()
        self.pool_size = pool_size
        
        # Statistics tracking
        self._stats_lock = threading.RLock()
        self._stats = {
            'total_requests': 0,
            'successful_acquisitions': 0,
            'timeouts': 0,
            'cancellations': 0,
            'average_wait_time': 0.0,
            'lock_utilization': 0.0
        }
        
        # Initialize database
        self._initialize_database()
        
        # Start cleanup thread
        self._cleanup_thread = None
        self._cleanup_stop_event = threading.Event()
        self._start_cleanup_thread()
        
        logger.info(f"ConflictBoxQueue initialized with database {db_name}@{db_host}:{db_port}")
    
    def _initialize_database(self) -> None:
        """Initialize database connection pool and create required tables."""
        try:
            # Create connection pool
            with self._pool_lock:
                self._connection_pool = psycopg2.pool.ThreadedConnectionPool(
                    minconn=2,
                    maxconn=self.pool_size,
                    **self.db_params
                )
            
            # Create tables and functions
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Create conflict box queue table
                    cur.execute("""
                        CREATE TABLE IF NOT EXISTS conflict_box_queue (
                            id SERIAL PRIMARY KEY,
                            box_id VARCHAR(50) NOT NULL,
                            robot_id VARCHAR(50) NOT NULL,
                            priority INTEGER NOT NULL DEFAULT 0,
                            queue_time TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
                            timeout_time TIMESTAMP NOT NULL,
                            position VARCHAR(20) NOT NULL DEFAULT 'queued',
                            estimated_wait_time REAL DEFAULT 0.0,
                            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            UNIQUE(box_id, robot_id)
                        )
                    """)
                    
                    # Create indexes for performance
                    cur.execute("""
                        CREATE INDEX IF NOT EXISTS idx_conflict_box_queue_box_id 
                        ON conflict_box_queue(box_id)
                    """)
                    cur.execute("""
                        CREATE INDEX IF NOT EXISTS idx_conflict_box_queue_robot_id 
                        ON conflict_box_queue(robot_id)
                    """)
                    cur.execute("""
                        CREATE INDEX IF NOT EXISTS idx_conflict_box_queue_priority_time 
                        ON conflict_box_queue(box_id, priority DESC, queue_time ASC)
                    """)
                    cur.execute("""
                        CREATE INDEX IF NOT EXISTS idx_conflict_box_queue_timeout 
                        ON conflict_box_queue(timeout_time)
                    """)
                    
                    # Create queue statistics table
                    cur.execute("""
                        CREATE TABLE IF NOT EXISTS conflict_box_queue_stats (
                            box_id VARCHAR(50) PRIMARY KEY,
                            total_requests INTEGER DEFAULT 0,
                            successful_acquisitions INTEGER DEFAULT 0,
                            timeouts INTEGER DEFAULT 0,
                            cancellations INTEGER DEFAULT 0,
                            total_wait_time REAL DEFAULT 0.0,
                            total_lock_time REAL DEFAULT 0.0,
                            last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                        )
                    """)
                    
                    # Create database functions for queue operations
                    cur.execute("""
                        CREATE OR REPLACE FUNCTION get_queue_position(p_box_id VARCHAR, p_robot_id VARCHAR)
                        RETURNS INTEGER AS $$
                        DECLARE
                            pos INTEGER;
                        BEGIN
                            SELECT COUNT(*) + 1 INTO pos
                            FROM conflict_box_queue
                            WHERE box_id = p_box_id
                            AND (priority > (SELECT priority FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)
                                 OR (priority = (SELECT priority FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)
                                     AND queue_time < (SELECT queue_time FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)));
                            RETURN pos;
                        END;
                        $$ LANGUAGE plpgsql;
                    """)
                    
                    conn.commit()
                    logger.info("Conflict box queue database schema initialized")
                    
        except psycopg2.Error as e:
            raise ConflictBoxQueueError(f"Database initialization failed: {e}")
        except Exception as e:
            raise ConflictBoxQueueError(f"Unexpected error during database initialization: {e}")
    
    @contextmanager
    def _get_connection(self):
        """Get database connection from pool with proper cleanup."""
        conn = None
        try:
            with self._pool_lock:
                if self._connection_pool is None:
                    raise ConflictBoxQueueError("Database connection pool not initialized")
                conn = self._connection_pool.getconn()
            
            if conn.closed:
                # Connection is closed, get a new one
                with self._pool_lock:
                    self._connection_pool.putconn(conn, close=True)
                    conn = self._connection_pool.getconn()
            
            yield conn
            
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            raise ConflictBoxQueueError(f"Database operation failed: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            raise ConflictBoxQueueError(f"Unexpected database error: {e}")
        finally:
            if conn:
                with self._pool_lock:
                    self._connection_pool.putconn(conn)
    
    def request_lock(self, box_id: str, robot_id: str, priority: int = 0, 
                    timeout_seconds: float = None) -> LockAcquisitionResult:
        """
        Request a lock on a conflict box with queue management.
        
        This method implements a comprehensive queue-based lock acquisition system:
        1. Check if lock is immediately available
        2. If not, add to priority queue
        3. Calculate estimated wait time
        4. Return appropriate result
        """
        if timeout_seconds is None:
            timeout_seconds = self.default_timeout
        
        timeout_time = time.time() + timeout_seconds
        
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Start transaction
                    cur.execute("BEGIN")
                    
                    # Check if robot already has a request for this box
                    cur.execute("""
                        SELECT position FROM conflict_box_queue
                        WHERE box_id = %s AND robot_id = %s
                    """, (box_id, robot_id))
                    
                    existing = cur.fetchone()
                    if existing:
                        if existing['position'] == 'lock_acquired':
                            # Robot already has the lock
                            conn.commit()
                            return LockAcquisitionResult(
                                success=True,
                                position=QueuePosition.LOCK_ACQUIRED,
                                estimated_wait_time=0.0,
                                queue_position=0
                            )
                        else:
                            # Robot already in queue, update timeout
                            cur.execute("""
                                UPDATE conflict_box_queue
                                SET timeout_time = %s, updated_at = CURRENT_TIMESTAMP
                                WHERE box_id = %s AND robot_id = %s
                            """, (timeout_time, box_id, robot_id))
                            conn.commit()
                            
                            # Get current position
                            position = self._get_queue_position(box_id, robot_id)
                            estimated_wait = self._calculate_estimated_wait_time(box_id, position)
                            
                            return LockAcquisitionResult(
                                success=False,
                                position=QueuePosition.QUEUED,
                                estimated_wait_time=estimated_wait,
                                queue_position=position
                            )
                    
                    # Check if lock is immediately available
                    cur.execute("""
                        SELECT locked_by_robot FROM conflict_box_locks
                        WHERE box_id = %s
                    """, (box_id,))
                    
                    current_lock = cur.fetchone()
                    if not current_lock:
                        # Lock is available, try to acquire it immediately
                        cur.execute("""
                            INSERT INTO conflict_box_locks (box_id, locked_by_robot, priority, locked_at, heartbeat_at)
                            VALUES (%s, %s, %s, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
                            ON CONFLICT (box_id) DO NOTHING
                        """, (box_id, robot_id, priority))
                        
                        if cur.rowcount > 0:
                            # Successfully acquired lock
                            cur.execute("""
                                INSERT INTO conflict_box_queue (box_id, robot_id, priority, timeout_time, position)
                                VALUES (%s, %s, %s, %s, 'lock_acquired')
                            """, (box_id, robot_id, priority, timeout_time))
                            
                            self._update_statistics('successful_acquisitions', 1)
                            conn.commit()
                            
                            logger.info(f"Lock acquired immediately: {box_id} by robot {robot_id}")
                            return LockAcquisitionResult(
                                success=True,
                                position=QueuePosition.LOCK_ACQUIRED,
                                estimated_wait_time=0.0,
                                queue_position=0
                            )
                    
                    # Lock not available, add to queue
                    cur.execute("""
                        INSERT INTO conflict_box_queue (box_id, robot_id, priority, timeout_time, position)
                        VALUES (%s, %s, %s, %s, 'queued')
                    """, (box_id, robot_id, priority, timeout_time))
                    
                    # Update queue positions
                    cur.execute("SELECT update_queue_positions_simple(%s)", (box_id,))
                    
                    # Get position and estimated wait time
                    position = self._get_queue_position(box_id, robot_id)
                    estimated_wait = self._calculate_estimated_wait_time(box_id, position)
                    
                    # Update estimated wait time in database
                    cur.execute("""
                        UPDATE conflict_box_queue
                        SET estimated_wait_time = %s, updated_at = CURRENT_TIMESTAMP
                        WHERE box_id = %s AND robot_id = %s
                    """, (estimated_wait, box_id, robot_id))
                    
                    self._update_statistics('total_requests', 1)
                    conn.commit()
                    
                    logger.info(f"Robot {robot_id} queued for {box_id} at position {position} (estimated wait: {estimated_wait:.1f}s)")
                    
                    return LockAcquisitionResult(
                        success=False,
                        position=QueuePosition.QUEUED,
                        estimated_wait_time=estimated_wait,
                        queue_position=position
                    )
                    
        except Exception as e:
            logger.error(f"Failed to request lock {box_id} for robot {robot_id}: {e}")
            raise ConflictBoxQueueError(f"Failed to request lock: {e}")
    
    def release_lock(self, box_id: str, robot_id: str) -> bool:
        """
        Release a conflict box lock and notify next robot in queue.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Start transaction
                    cur.execute("BEGIN")
                    
                    # Verify robot owns the lock
                    cur.execute("""
                        SELECT locked_by_robot FROM conflict_box_locks
                        WHERE box_id = %s AND locked_by_robot = %s
                    """, (box_id, robot_id))
                    
                    if not cur.fetchone():
                        conn.rollback()
                        logger.warning(f"Robot {robot_id} tried to release lock {box_id} but doesn't own it")
                        return False
                    
                    # Record lock duration for statistics
                    cur.execute("""
                        SELECT EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - locked_at)) as duration
                        FROM conflict_box_locks
                        WHERE box_id = %s AND locked_by_robot = %s
                    """, (box_id, robot_id))
                    
                    duration_result = cur.fetchone()
                    lock_duration = duration_result['duration'] if duration_result else 0.0
                    
                    # Release the lock
                    cur.execute("""
                        DELETE FROM conflict_box_locks
                        WHERE box_id = %s AND locked_by_robot = %s
                    """, (box_id, robot_id))
                    
                    # Remove from queue
                    cur.execute("""
                        DELETE FROM conflict_box_queue
                        WHERE box_id = %s AND robot_id = %s
                    """, (box_id, robot_id))
                    
                    # Get next robot in queue
                    cur.execute("""
                        SELECT robot_id, priority FROM conflict_box_queue
                        WHERE box_id = %s AND position != 'timeout' AND position != 'cancelled'
                        ORDER BY priority DESC, queue_time ASC
                        LIMIT 1
                    """, (box_id,))
                    
                    next_robot = cur.fetchone()
                    if next_robot:
                        next_robot_id = next_robot['robot_id']
                        next_priority = next_robot['priority']
                        
                        # Acquire lock for next robot
                        cur.execute("""
                            INSERT INTO conflict_box_locks (box_id, locked_by_robot, priority, locked_at, heartbeat_at)
                            VALUES (%s, %s, %s, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
                        """, (box_id, next_robot_id, next_priority))
                        
                        # Update queue position
                        cur.execute("""
                            UPDATE conflict_box_queue
                            SET position = 'lock_acquired', updated_at = CURRENT_TIMESTAMP
                            WHERE box_id = %s AND robot_id = %s
                        """, (box_id, next_robot_id))
                        
                        logger.info(f"Lock transferred from {robot_id} to {next_robot_id} for {box_id}")
                    
                    # Update queue positions for remaining robots
                    cur.execute("SELECT update_queue_positions_simple(%s)", (box_id,))
                    
                    # Update statistics
                    self._update_lock_statistics(box_id, lock_duration)
                    
                    conn.commit()
                    logger.info(f"Lock released: {box_id} by robot {robot_id}")
                    return True
                    
        except Exception as e:
            logger.error(f"Failed to release lock {box_id} for robot {robot_id}: {e}")
            raise ConflictBoxQueueError(f"Failed to release lock: {e}")
    
    def cancel_request(self, box_id: str, robot_id: str) -> bool:
        """
        Cancel a pending lock request.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Update position to cancelled
                    cur.execute("""
                        UPDATE conflict_box_queue
                        SET position = 'cancelled', updated_at = CURRENT_TIMESTAMP
                        WHERE box_id = %s AND robot_id = %s AND position IN ('queued', 'next_in_line')
                    """, (box_id, robot_id))
                    
                    if cur.rowcount > 0:
                        # Update queue positions
                        cur.execute("SELECT update_queue_positions_simple(%s)", (box_id,))
                        
                        self._update_statistics('cancellations', 1)
                        conn.commit()
                        
                        logger.info(f"Request cancelled: {box_id} by robot {robot_id}")
                        return True
                    else:
                        logger.warning(f"No pending request found to cancel: {box_id} by robot {robot_id}")
                        return False
                        
        except Exception as e:
            logger.error(f"Failed to cancel request {box_id} for robot {robot_id}: {e}")
            raise ConflictBoxQueueError(f"Failed to cancel request: {e}")
    
    def get_queue_status(self, box_id: str) -> QueueStatus:
        """
        Get current status of a conflict box queue.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Get current lock owner
                    cur.execute("""
                        SELECT locked_by_robot FROM conflict_box_locks
                        WHERE box_id = %s
                    """, (box_id,))
                    
                    lock_result = cur.fetchone()
                    current_owner = lock_result['locked_by_robot'] if lock_result else None
                    
                    # Get queue information
                    cur.execute("""
                        SELECT COUNT(*) as queue_length,
                               MIN(CASE WHEN position = 'next_in_line' THEN robot_id END) as next_robot
                        FROM conflict_box_queue
                        WHERE box_id = %s AND position IN ('queued', 'next_in_line')
                    """, (box_id,))
                    
                    queue_result = cur.fetchone()
                    queue_length = queue_result['queue_length'] if queue_result else 0
                    next_robot = queue_result['next_robot'] if queue_result else None
                    
                    # Get statistics
                    cur.execute("""
                        SELECT total_wait_time, successful_acquisitions,
                               total_lock_time, total_requests
                        FROM conflict_box_queue_stats
                        WHERE box_id = %s
                    """, (box_id,))
                    
                    stats_result = cur.fetchone()
                    if stats_result and stats_result['successful_acquisitions'] > 0:
                        avg_wait_time = stats_result['total_wait_time'] / stats_result['successful_acquisitions']
                        avg_lock_time = stats_result['total_lock_time'] / stats_result['successful_acquisitions']
                        lock_duration_stats = {
                            'avg': avg_lock_time,
                            'min': 0.0,  # Would need additional tracking for min/max
                            'max': 0.0
                        }
                    else:
                        avg_wait_time = 0.0
                        lock_duration_stats = {'avg': 0.0, 'min': 0.0, 'max': 0.0}
                    
                    return QueueStatus(
                        box_id=box_id,
                        current_owner=current_owner,
                        queue_length=queue_length,
                        next_robot=next_robot,
                        average_wait_time=avg_wait_time,
                        lock_duration_stats=lock_duration_stats
                    )
                    
        except Exception as e:
            logger.error(f"Failed to get queue status for {box_id}: {e}")
            raise ConflictBoxQueueError(f"Failed to get queue status: {e}")
    
    def get_robot_queue_position(self, box_id: str, robot_id: str) -> Optional[QueueEntry]:
        """
        Get a robot's position in a specific queue.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT robot_id, box_id, priority, 
                               EXTRACT(EPOCH FROM queue_time) as queue_time,
                               EXTRACT(EPOCH FROM timeout_time) as timeout_time,
                               position, estimated_wait_time
                        FROM conflict_box_queue
                        WHERE box_id = %s AND robot_id = %s
                    """, (box_id, robot_id))
                    
                    result = cur.fetchone()
                    if not result:
                        return None
                    
                    return QueueEntry(
                        robot_id=result['robot_id'],
                        box_id=result['box_id'],
                        priority=result['priority'],
                        queue_time=result['queue_time'],
                        timeout_time=result['timeout_time'],
                        position=QueuePosition(result['position']),
                        estimated_wait_time=result['estimated_wait_time']
                    )
                    
        except Exception as e:
            logger.error(f"Failed to get robot queue position for {robot_id} in {box_id}: {e}")
            raise ConflictBoxQueueError(f"Failed to get robot queue position: {e}")
    
    def get_robot_queues(self, robot_id: str) -> List[QueueEntry]:
        """
        Get all queues that a robot is currently in.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT robot_id, box_id, priority, 
                               EXTRACT(EPOCH FROM queue_time) as queue_time,
                               EXTRACT(EPOCH FROM timeout_time) as timeout_time,
                               position, estimated_wait_time
                        FROM conflict_box_queue
                        WHERE robot_id = %s AND position NOT IN ('timeout', 'cancelled')
                        ORDER BY priority DESC, queue_time ASC
                    """, (robot_id,))
                    
                    results = cur.fetchall()
                    return [
                        QueueEntry(
                            robot_id=row['robot_id'],
                            box_id=row['box_id'],
                            priority=row['priority'],
                            queue_time=row['queue_time'],
                            timeout_time=row['timeout_time'],
                            position=QueuePosition(row['position']),
                            estimated_wait_time=row['estimated_wait_time']
                        )
                        for row in results
                    ]
                    
        except Exception as e:
            logger.error(f"Failed to get robot queues for {robot_id}: {e}")
            raise ConflictBoxQueueError(f"Failed to get robot queues: {e}")
    
    def heartbeat_lock(self, box_id: str, robot_id: str) -> bool:
        """
        Send heartbeat for an active lock.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        UPDATE conflict_box_locks
                        SET heartbeat_at = CURRENT_TIMESTAMP
                        WHERE box_id = %s AND locked_by_robot = %s
                    """, (box_id, robot_id))
                    
                    success = cur.rowcount > 0
                    conn.commit()
                    
                    if success:
                        logger.debug(f"Heartbeat sent for lock {box_id} by robot {robot_id}")
                    else:
                        logger.warning(f"Failed to send heartbeat for lock {box_id} by robot {robot_id}")
                    
                    return success
                    
        except Exception as e:
            logger.error(f"Failed to send heartbeat for lock {box_id} by robot {robot_id}: {e}")
            raise ConflictBoxQueueError(f"Failed to send heartbeat: {e}")
    
    def cleanup_expired_requests(self) -> int:
        """
        Remove expired queue entries and stale locks.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Mark expired requests as timeout
                    cur.execute("""
                        UPDATE conflict_box_queue
                        SET position = 'timeout', updated_at = CURRENT_TIMESTAMP
                        WHERE timeout_time < CURRENT_TIMESTAMP
                        AND position IN ('queued', 'next_in_line')
                    """)
                    
                    timeout_count = cur.rowcount
                    
                    # Clean up expired locks (no heartbeat for too long)
                    cur.execute("""
                        DELETE FROM conflict_box_locks
                        WHERE heartbeat_at < CURRENT_TIMESTAMP - INTERVAL '%s seconds'
                    """, (self.heartbeat_interval * 3,))  # 3x heartbeat interval
                    
                    expired_locks = cur.rowcount
                    
                    # Remove old timeout/cancelled entries
                    cur.execute("""
                        DELETE FROM conflict_box_queue
                        WHERE position IN ('timeout', 'cancelled')
                        AND updated_at < CURRENT_TIMESTAMP - INTERVAL '1 hour'
                    """)
                    
                    old_entries = cur.rowcount
                    
                    # Update statistics
                    if timeout_count > 0:
                        self._update_statistics('timeouts', timeout_count)
                    
                    conn.commit()
                    
                    total_cleaned = timeout_count + expired_locks + old_entries
                    if total_cleaned > 0:
                        logger.info(f"Cleaned up {total_cleaned} expired entries "
                                  f"({timeout_count} timeouts, {expired_locks} stale locks, {old_entries} old entries)")
                    
                    return total_cleaned
                    
        except Exception as e:
            logger.error(f"Failed to cleanup expired requests: {e}")
            raise ConflictBoxQueueError(f"Failed to cleanup expired requests: {e}")
    
    def get_queue_statistics(self) -> Dict[str, Any]:
        """
        Get comprehensive queue statistics.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Get current queue counts
                    cur.execute("""
                        SELECT 
                            COUNT(DISTINCT box_id) as total_queues,
                            COUNT(*) as total_waiting_robots,
                            AVG(estimated_wait_time) as avg_estimated_wait,
                            COUNT(CASE WHEN priority = 0 THEN 1 END) as priority_0,
                            COUNT(CASE WHEN priority = 1 THEN 1 END) as priority_1,
                            COUNT(CASE WHEN priority >= 2 THEN 1 END) as priority_high
                        FROM conflict_box_queue
                        WHERE position IN ('queued', 'next_in_line')
                    """)
                    
                    current_stats = cur.fetchone()
                    
                    # Get historical statistics
                    cur.execute("""
                        SELECT 
                            SUM(total_requests) as total_requests,
                            SUM(successful_acquisitions) as successful_acquisitions,
                            SUM(timeouts) as timeouts,
                            SUM(cancellations) as cancellations,
                            AVG(CASE WHEN successful_acquisitions > 0 
                                THEN total_wait_time / successful_acquisitions 
                                ELSE 0 END) as avg_wait_time,
                            AVG(CASE WHEN successful_acquisitions > 0 
                                THEN total_lock_time / successful_acquisitions 
                                ELSE 0 END) as avg_lock_time
                        FROM conflict_box_queue_stats
                    """)
                    
                    historical_stats = cur.fetchone()
                    
                    # Get lock utilization
                    cur.execute("""
                        SELECT COUNT(*) as active_locks
                        FROM conflict_box_locks
                    """)
                    
                    lock_stats = cur.fetchone()
                    
                    return {
                        'total_queues': current_stats['total_queues'] or 0,
                        'total_waiting_robots': current_stats['total_waiting_robots'] or 0,
                        'active_locks': lock_stats['active_locks'] or 0,
                        'average_wait_time': historical_stats['avg_wait_time'] or 0.0,
                        'average_lock_time': historical_stats['avg_lock_time'] or 0.0,
                        'lock_utilization': 0.0,  # Would need more complex calculation
                        'timeout_rate': (
                            (historical_stats['timeouts'] or 0) / 
                            max(1, historical_stats['total_requests'] or 1)
                        ) * 100.0,
                        'success_rate': (
                            (historical_stats['successful_acquisitions'] or 0) / 
                            max(1, historical_stats['total_requests'] or 1)
                        ) * 100.0,
                        'priority_distribution': {
                            'priority_0': current_stats['priority_0'] or 0,
                            'priority_1': current_stats['priority_1'] or 0,
                            'priority_high': current_stats['priority_high'] or 0
                        },
                        'estimated_wait_time': current_stats['avg_estimated_wait'] or 0.0
                    }
                    
        except Exception as e:
            logger.error(f"Failed to get queue statistics: {e}")
            raise ConflictBoxQueueError(f"Failed to get queue statistics: {e}")
    
    def _get_queue_position(self, box_id: str, robot_id: str) -> int:
        """Get robot's position in queue (1-based)."""
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("SELECT get_queue_position(%s, %s)", (box_id, robot_id))
                    result = cur.fetchone()
                    return result[0] if result else 0
        except Exception:
            return 0
    
    def _calculate_estimated_wait_time(self, box_id: str, position: int) -> float:
        """Calculate estimated wait time based on position and historical data."""
        if position <= 1:
            return 0.0
        
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Get average lock duration for this box
                    cur.execute("""
                        SELECT CASE WHEN successful_acquisitions > 0 
                               THEN total_lock_time / successful_acquisitions 
                               ELSE 30.0 END as avg_lock_time
                        FROM conflict_box_queue_stats
                        WHERE box_id = %s
                    """, (box_id,))
                    
                    result = cur.fetchone()
                    avg_lock_time = result['avg_lock_time'] if result else 30.0
                    
                    # Estimate: (position - 1) * average_lock_time
                    return (position - 1) * avg_lock_time
                    
        except Exception:
            # Fallback: assume 30 seconds per position
            return (position - 1) * 30.0
    
    def _update_statistics(self, stat_name: str, increment: int) -> None:
        """Update internal statistics."""
        with self._stats_lock:
            self._stats[stat_name] += increment
    
    def _update_lock_statistics(self, box_id: str, lock_duration: float) -> None:
        """Update lock statistics in database."""
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        INSERT INTO conflict_box_queue_stats 
                        (box_id, total_lock_time, successful_acquisitions, last_updated)
                        VALUES (%s, %s, 1, CURRENT_TIMESTAMP)
                        ON CONFLICT (box_id) DO UPDATE SET
                        total_lock_time = conflict_box_queue_stats.total_lock_time + %s,
                        successful_acquisitions = conflict_box_queue_stats.successful_acquisitions + 1,
                        last_updated = CURRENT_TIMESTAMP
                    """, (box_id, lock_duration, lock_duration))
                    
                    conn.commit()
        except Exception as e:
            logger.error(f"Failed to update lock statistics for {box_id}: {e}")
    
    def _start_cleanup_thread(self) -> None:
        """Start the cleanup thread."""
        if self._cleanup_thread is None or not self._cleanup_thread.is_alive():
            self._cleanup_thread = threading.Thread(
                target=self._cleanup_loop,
                name="ConflictBoxQueueCleanup",
                daemon=True
            )
            self._cleanup_thread.start()
            logger.info("Cleanup thread started")
    
    def _cleanup_loop(self) -> None:
        """Main cleanup loop."""
        while not self._cleanup_stop_event.is_set():
            try:
                self.cleanup_expired_requests()
            except Exception as e:
                logger.error(f"Error in cleanup loop: {e}")
            
            self._cleanup_stop_event.wait(self.cleanup_interval)
    
    def close(self) -> None:
        """Close the queue system and cleanup resources."""
        try:
            # Stop cleanup thread
            if self._cleanup_thread and self._cleanup_thread.is_alive():
                self._cleanup_stop_event.set()
                self._cleanup_thread.join(timeout=5.0)
            
            # Close database connections
            if self._connection_pool:
                with self._pool_lock:
                    self._connection_pool.closeall()
                    self._connection_pool = None
                    
            logger.info("ConflictBoxQueue closed")
        except Exception as e:
            logger.error(f"Error closing ConflictBoxQueue: {e}")
    
    def __del__(self):
        """Ensure cleanup on object destruction."""
        self.close()


class ConflictBoxQueueNotifierImpl(IConflictBoxQueueNotifier):
    """
    Implementation of conflict box queue notifications.
    
    This provides a simple callback-based notification system for queue events.
    In a more advanced implementation, this could use message queues, WebSockets,
    or other asynchronous communication mechanisms.
    """
    
    def __init__(self):
        """Initialize the notifier."""
        self._callbacks: Dict[str, Callable] = {}
        self._callbacks_lock = threading.RLock()
        logger.info("ConflictBoxQueueNotifier initialized")
    
    def notify_lock_available(self, box_id: str, robot_id: str) -> None:
        """Notify a robot that a lock is now available."""
        self._send_notification(robot_id, 'lock_available', {
            'box_id': box_id,
            'timestamp': time.time()
        })
    
    def notify_queue_position_changed(self, box_id: str, robot_id: str, 
                                    new_position: int, estimated_wait_time: float) -> None:
        """Notify a robot that their queue position has changed."""
        self._send_notification(robot_id, 'queue_position_changed', {
            'box_id': box_id,
            'new_position': new_position,
            'estimated_wait_time': estimated_wait_time,
            'timestamp': time.time()
        })
    
    def notify_request_timeout(self, box_id: str, robot_id: str) -> None:
        """Notify a robot that their lock request has timed out."""
        self._send_notification(robot_id, 'request_timeout', {
            'box_id': box_id,
            'timestamp': time.time()
        })
    
    def register_callback(self, robot_id: str, callback: Callable) -> None:
        """Register a callback function for queue notifications."""
        with self._callbacks_lock:
            self._callbacks[robot_id] = callback
        logger.debug(f"Registered callback for robot {robot_id}")
    
    def unregister_callback(self, robot_id: str) -> None:
        """Unregister a callback function."""
        with self._callbacks_lock:
            self._callbacks.pop(robot_id, None)
        logger.debug(f"Unregistered callback for robot {robot_id}")
    
    def _send_notification(self, robot_id: str, event_type: str, data: Dict[str, Any]) -> None:
        """Send a notification to a robot."""
        try:
            with self._callbacks_lock:
                callback = self._callbacks.get(robot_id)
            
            if callback:
                callback(event_type, data)
                logger.debug(f"Sent {event_type} notification to robot {robot_id}")
            else:
                logger.debug(f"No callback registered for robot {robot_id}")
                
        except Exception as e:
            logger.error(f"Failed to send notification to robot {robot_id}: {e}")