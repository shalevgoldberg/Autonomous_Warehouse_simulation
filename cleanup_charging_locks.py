#!/usr/bin/env python3
"""
Safe Cleanup of Orphaned Charging Bay Locks
==========================================

This script safely removes ONLY expired charging bay locks with confirmation.
It follows these safety principles:
- READ-ONLY first: Just SELECT to see what's there
- VERIFY before DELETE: Check if locks are truly expired
- TRANSACTION safety: Use transactions for all operations
- CHARGING-ONLY: Only touches charging bay locks (charge_*)
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
        database="warehouse_sim",
        user="postgres",
        password=password,
        cursor_factory=RealDictCursor,
        connect_timeout=10,
        application_name='charging_locks_cleanup'
    )

def check_expired_charging_locks():
    """Step 1: Check for expired charging bay locks only."""
    logger.info("🔍 STEP 1: Checking expired charging bay locks...")

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                # Get only expired charging bay locks
                cur.execute("""
                    SELECT
                        bay_id,
                        robot_id,
                        locked_at,
                        heartbeat_at,
                        lock_timeout_seconds,
                        EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_at)) as seconds_since_heartbeat
                    FROM bay_locks
                    WHERE bay_id LIKE 'charge_%'
                    AND CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds)
                    ORDER BY heartbeat_at DESC
                """)

                expired_locks = cur.fetchall()

                if not expired_locks:
                    logger.info("✅ No expired charging bay locks found")
                    return []

                logger.info(f"⚠️  Found {len(expired_locks)} EXPIRED charging bay locks:")
                for lock in expired_locks:
                    logger.warning(f"   🗑️  Bay: {lock['bay_id']} | Robot: {lock['robot_id']} | "
                                 f"Expired: {lock['seconds_since_heartbeat']:.1f}s ago")

                return expired_locks

    except Exception as e:
        logger.error(f"❌ Error checking expired charging locks: {e}")
        return None

def get_active_charging_locks():
    """Check for any active (non-expired) charging locks to ensure we don't affect them."""
    logger.info("🔍 STEP 2: Checking for active charging bay locks...")

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("""
                    SELECT
                        bay_id,
                        robot_id,
                        locked_at,
                        heartbeat_at,
                        lock_timeout_seconds,
                        EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_at)) as seconds_since_heartbeat
                    FROM bay_locks
                    WHERE bay_id LIKE 'charge_%'
                    AND CURRENT_TIMESTAMP <= (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds)
                    ORDER BY heartbeat_at DESC
                """)

                active_locks = cur.fetchall()

                if active_locks:
                    logger.info(f"✅ Found {len(active_locks)} ACTIVE charging bay locks (will NOT be touched):")
                    for lock in active_locks:
                        logger.info(f"   ✅ Bay: {lock['bay_id']} | Robot: {lock['robot_id']} | "
                                  f"Heartbeat: {lock['seconds_since_heartbeat']:.1f}s ago")
                else:
                    logger.info("ℹ️  No active charging bay locks found")

                return active_locks

    except Exception as e:
        logger.error(f"❌ Error checking active charging locks: {e}")
        return None

def safe_cleanup_expired_charging_locks(expired_locks):
    """Step 3: Safely cleanup expired charging locks with transaction rollback capability."""
    if not expired_locks:
        logger.info("ℹ️  No expired charging locks to clean up")
        return True

    logger.warning(f"⚠️  PREPARING TO DELETE {len(expired_locks)} EXPIRED CHARGING BAY LOCKS")
    logger.warning("🛑 MANUAL CONFIRMATION REQUIRED")

    # Show what will be deleted
    logger.info("📋 Locks scheduled for deletion:")
    for i, lock in enumerate(expired_locks, 1):
        logger.info(f"   {i}. Bay: {lock['bay_id']} | Robot: {lock['robot_id']}")

    # Ask for confirmation
    print("\n" + "="*60)
    response = input("❓ Do you want to proceed with cleanup of EXPIRED CHARGING LOCKS only? Type 'yes' to confirm: ")
    if response.lower() != 'yes':
        logger.info("❌ Cleanup cancelled by user")
        return False

    # Double confirmation
    response2 = input("❓ Are you ABSOLUTELY sure? This will delete the expired locks. Type 'confirm' to proceed: ")
    if response2.lower() != 'confirm':
        logger.info("❌ Cleanup cancelled by user (second confirmation)")
        return False

    # Proceed with cleanup
    logger.info("🧹 STEP 3: Cleaning up expired charging locks...")

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                # Start transaction
                cur.execute("BEGIN")

                deleted_count = 0
                for lock in expired_locks:
                    # Double-check the lock is still expired before deleting
                    cur.execute("""
                        DELETE FROM bay_locks
                        WHERE bay_id = %s AND robot_id = %s
                        AND bay_id LIKE 'charge_%%'
                        AND CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds)
                    """, (lock['bay_id'], lock['robot_id']))

                    if cur.rowcount > 0:
                        deleted_count += 1
                        logger.info(f"   ✅ Deleted expired charging lock: {lock['bay_id']}")
                    else:
                        logger.warning(f"   ⚠️  Lock no longer exists or no longer expired: {lock['bay_id']}")

                # Commit transaction
                conn.commit()
                logger.info(f"✅ Successfully cleaned up {deleted_count} expired charging bay locks")

                # Verify cleanup - check remaining charging locks
                cur.execute("SELECT COUNT(*) as remaining FROM bay_locks WHERE bay_id LIKE 'charge_%'")
                remaining_charging = cur.fetchone()['remaining']
                logger.info(f"📊 Remaining charging bay locks: {remaining_charging}")

                # Show remaining locks if any
                if remaining_charging > 0:
                    cur.execute("""
                        SELECT bay_id, robot_id, locked_at,
                               EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_at)) as seconds_since_heartbeat,
                               CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds) as is_expired
                        FROM bay_locks
                        WHERE bay_id LIKE 'charge_%'
                        ORDER BY locked_at DESC
                    """)
                    remaining_locks = cur.fetchall()
                    logger.info("🔍 Remaining charging locks:")
                    for lock in remaining_locks:
                        status = "❌ EXPIRED" if lock['is_expired'] else "✅ ACTIVE"
                        logger.info(f"   {status} | Bay: {lock['bay_id']} | Robot: {lock['robot_id']}")

                return True

    except Exception as e:
        logger.error(f"❌ Error during cleanup: {e}")
        logger.error("🔄 Transaction rolled back - no changes made to database")
        return False

def main():
    """Main function with safety checks."""
    logger.info("🚀 Starting Safe Charging Bay Locks Cleanup")
    logger.info("=" * 60)

    # Step 1: Check expired charging locks
    expired_locks = check_expired_charging_locks()
    if expired_locks is None:
        logger.error("❌ Failed to check expired charging locks")
        return False

    if not expired_locks:
        logger.info("✅ No expired charging locks found - cleanup not needed")
        return True

    # Step 2: Check active charging locks (for safety)
    active_locks = get_active_charging_locks()
    if active_locks is None:
        logger.error("❌ Failed to check active charging locks")
        return False

    # Step 3: Safe cleanup
    success = safe_cleanup_expired_charging_locks(expired_locks)
    return success

if __name__ == "__main__":
    try:
        success = main()
        if success:
            logger.info("✅ Charging bay locks cleanup completed successfully")
            sys.exit(0)
        else:
            logger.error("❌ Charging bay locks cleanup failed")
            sys.exit(1)
    except KeyboardInterrupt:
        logger.info("⏹️  Operation cancelled by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"💥 Unexpected error: {e}")
        sys.exit(1)
