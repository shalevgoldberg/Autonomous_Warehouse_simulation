#!/usr/bin/env python3
"""
Simple Bay Locks Database Check
"""

import os
import psycopg2
from psycopg2.extras import RealDictCursor
from datetime import datetime

password = os.getenv('WAREHOUSE_DB_PASSWORD')
if not password:
    print('❌ WAREHOUSE_DB_PASSWORD not set')
    exit(1)

print('🔗 Testing database connection...')
try:
    conn = psycopg2.connect(
        host='localhost',
        port=5432,
        database='warehouse_sim',
        user='postgres',
        password=password,
        connect_timeout=5,
        cursor_factory=RealDictCursor
    )
    print('✅ Database connection successful')

    with conn.cursor() as cur:
        # Check if bay_locks table exists
        cur.execute("""
            SELECT EXISTS (
                SELECT 1 FROM information_schema.tables
                WHERE table_name = 'bay_locks'
            )
        """)
        table_exists = cur.fetchone()['exists']

        if not table_exists:
            print('❌ bay_locks table does not exist in database')
            conn.close()
            exit(1)

        # Check bay locks
        cur.execute('SELECT COUNT(*) as lock_count FROM bay_locks')
        result = cur.fetchone()
        print(f'📊 Current bay locks in database: {result["lock_count"]}')

        if result['lock_count'] > 0:
            try:
                cur.execute("""
                    SELECT
                        bay_id,
                        robot_id,
                        locked_at,
                        heartbeat_at,
                        lock_timeout_seconds,
                        EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_at)) as seconds_since_heartbeat,
                        CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds) as is_expired
                    FROM bay_locks
                    ORDER BY locked_at DESC
                """)
                locks = cur.fetchall()
                print('🔍 Bay locks details:')
                charging_locks = 0
                for lock in locks:
                    status = "❌ EXPIRED" if lock['is_expired'] else "✅ ACTIVE"
                    bay_type = "🔋 CHARGING" if lock["bay_id"].startswith("charge_") else "🅿️  IDLE"
                    if lock["bay_id"].startswith("charge_"):
                        charging_locks += 1
                    print(f'   {status} | {bay_type} | Bay: {lock["bay_id"]} | Robot: {lock["robot_id"]}')
                    print(f'       Last heartbeat: {lock["seconds_since_heartbeat"]:.1f}s ago | Timeout: {lock["lock_timeout_seconds"]}s')
                print(f'🔋 Charging bay locks: {charging_locks} out of {len(locks)} total')
            except Exception as e:
                print(f'❌ Error querying bay locks: {e}')
                # Try basic select without computed columns
                cur.execute('SELECT bay_id, robot_id, locked_at FROM bay_locks ORDER BY locked_at DESC')
                basic_locks = cur.fetchall()
                print('🔍 Basic bay locks info (without computed fields):')
                for lock in basic_locks:
                    print(f'   Bay: {lock["bay_id"]} | Robot: {lock["robot_id"]} | Locked at: {lock["locked_at"]}')
        else:
            print('✅ No bay locks found - database appears clean')

    conn.close()

except Exception as e:
    print(f'❌ Database connection failed: {e}')
    import traceback
    traceback.print_exc()
