#!/usr/bin/env python3
"""
Safe Database Bay Locks Checker and Cleaner
===========================================

This script safely checks for orphaned bay locks in the database and cleans them up.
It follows these safety principles:
- READ-ONLY first: Just SELECT to see what's there
- VERIFY before DELETE: Check if locks are truly orphaned
- TRANSACTION safety: Use transactions for all operations
- NO schema changes: Only touches data, not structure
"""

import os
import sys
import psycopg2
from psycopg2.extras import RealDictCursor
from datetime import datetime, timedelta
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def get_db_connection():
    """Get database connection using environment variables."""
    password = os.getenv('WAREHOUSE_DB_PASSWORD')
    if not password:
        raise ValueError("WAREHOUSE_DB_PASSWORD environment variable not set")

    return psycopg2.connect(
        host="localhost",
        port=5432,
        database="warehouse_db",
        user="warehouse_user",
        password=password,
        cursor_factory=RealDictCursor,
        connect_timeout=10,
        application_name='bay_locks_checker'
    )

def check_bay_locks_readonly():
    """Step 1: READ-ONLY check of current bay locks."""
    logger.info("🔍 STEP 1: Checking current bay locks (READ-ONLY)...")

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                # Get all current bay locks
                cur.execute("""
                    SELECT
                        bay_id,
                        locked_by_robot,
                        lock_timestamp,
                        heartbeat_timestamp,
                        lock_timeout_seconds,
                        EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_timestamp)) as seconds_since_heartbeat,
                        CURRENT_TIMESTAMP > (heartbeat_timestamp + INTERVAL '1 second' * lock_timeout_seconds) as is_expired
                    FROM bay_locks
                    ORDER BY lock_timestamp DESC
                """)

                locks = cur.fetchall()

                if not locks:
                    logger.info("✅ No bay locks found in database")
                    return []

                logger.info(f"📋 Found {len(locks)} bay locks:")
                for lock in locks:
                    status = "❌ EXPIRED" if lock['is_expired'] else "✅ ACTIVE"
                    logger.info(f"   {status} | Bay: {lock['bay_id']} | Robot: {lock['locked_by_robot']} | "
                              f"Heartbeat: {lock['seconds_since_heartbeat']:.1f}s ago | "
                              f"Timeout: {lock['lock_timeout_seconds']}s")

                return locks

    except Exception as e:
        logger.error(f"❌ Error checking bay locks: {e}")
        return None

def verify_orphaned_locks(locks):
    """Step 2: Verify which locks are truly orphaned."""
    logger.info("🔍 STEP 2: Verifying orphaned locks...")

    if not locks:
        return []

    orphaned_locks = []
    for lock in locks:
        if lock['is_expired']:
            orphaned_locks.append(lock)
            logger.info(f"   🗑️  ORPHANED: {lock['bay_id']} (expired {lock['seconds_since_heartbeat']:.1f}s ago)")
        else:
            logger.info(f"   ✅ ACTIVE: {lock['bay_id']} (still valid)")

    if not orphaned_locks:
        logger.info("✅ No orphaned locks found - all locks are still active")

    return orphaned_locks

def cleanup_orphaned_locks_safe(orphaned_locks):
    """Step 3: Safely cleanup orphaned locks with transaction rollback capability."""
    if not orphaned_locks:
        logger.info("ℹ️  No orphaned locks to clean up")
        return

    logger.warning(f"⚠️  FOUND {len(orphaned_locks)} ORPHANED BAY LOCKS")
    logger.warning("🛑 CLEANUP REQUIRES MANUAL CONFIRMATION")

    # Show what will be deleted
    logger.info("📋 Locks to be deleted:")
    for lock in orphaned_locks:
        logger.info(f"   🗑️  Bay: {lock['bay_id']} | Robot: {lock['locked_by_robot']} | "
                  f"Expired: {lock['seconds_since_heartbeat']:.1f}s ago")

    # Ask for confirmation
    response = input("\n❓ Do you want to proceed with cleanup? Type 'yes' to confirm: ")
    if response.lower() != 'yes':
        logger.info("❌ Cleanup cancelled by user")
        return

    # Proceed with cleanup
    logger.info("🧹 STEP 3: Cleaning up orphaned locks...")

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                # Start transaction
                cur.execute("BEGIN")

                deleted_count = 0
                for lock in orphaned_locks:
                    # Double-check the lock is still expired before deleting
                    cur.execute("""
                        DELETE FROM bay_locks
                        WHERE bay_id = %s AND locked_by_robot = %s
                        AND CURRENT_TIMESTAMP > (heartbeat_timestamp + INTERVAL '1 second' * lock_timeout_seconds)
                    """, (lock['bay_id'], lock['locked_by_robot']))

                    if cur.rowcount > 0:
                        deleted_count += 1
                        logger.info(f"   ✅ Deleted orphaned lock: {lock['bay_id']}")
                    else:
                        logger.warning(f"   ⚠️  Lock no longer exists or no longer expired: {lock['bay_id']}")

                # Commit transaction
                conn.commit()
                logger.info(f"✅ Successfully cleaned up {deleted_count} orphaned bay locks")

                # Verify cleanup
                cur.execute("SELECT COUNT(*) as remaining FROM bay_locks")
                remaining = cur.fetchone()['remaining']
                logger.info(f"📊 Remaining bay locks after cleanup: {remaining}")

    except Exception as e:
        logger.error(f"❌ Error during cleanup: {e}")
        logger.error("🔄 Transaction rolled back - no changes made to database")
        return False

    return True

def main():
    """Main function with safety checks."""
    logger.info("🚀 Starting Bay Locks Database Checker and Cleaner")
    logger.info("=" * 60)

    # Step 1: Read-only check
    locks = check_bay_locks_readonly()
    if locks is None:
        logger.error("❌ Failed to read bay locks from database")
        return False

    if not locks:
        logger.info("✅ Database is clean - no bay locks to check")
        return True

    # Step 2: Verify orphaned locks
    orphaned_locks = verify_orphaned_locks(locks)
    if not orphaned_locks:
        logger.info("✅ All bay locks are active - no cleanup needed")
        return True

    # Step 3: Safe cleanup (requires user confirmation)
    success = cleanup_orphaned_locks_safe(orphaned_locks)
    return success

if __name__ == "__main__":
    try:
        success = main()
        if success:
            logger.info("✅ Bay locks check/cleanup completed successfully")
            sys.exit(0)
        else:
            logger.error("❌ Bay locks check/cleanup failed")
            sys.exit(1)
    except KeyboardInterrupt:
        logger.info("⏹️  Operation cancelled by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"💥 Unexpected error: {e}")
        sys.exit(1)
