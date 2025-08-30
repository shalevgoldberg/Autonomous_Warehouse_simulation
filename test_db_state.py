#!/usr/bin/env python3
"""
Test database state to check bay locks
"""
import os
os.environ['WAREHOUSE_DB_PASSWORD'] = 'renaspolter'

import psycopg2

print('🔍 Testing database connection and bay_locks table...')

try:
    conn = psycopg2.connect(
        host='localhost',
        port=5432,
        database='warehouse_sim',
        user='postgres',
        password='renaspolter'
    )

    with conn.cursor() as cur:
        # Check if bay_locks table exists
        cur.execute("""
            SELECT EXISTS (
                SELECT 1 FROM information_schema.tables
                WHERE table_schema = 'public' AND table_name = 'bay_locks'
            )
        """)
        exists = cur.fetchone()[0]
        print(f'📋 bay_locks table exists: {exists}')

        if exists:
            # Check table structure
            cur.execute("""
                SELECT column_name, data_type, column_default, is_nullable
                FROM information_schema.columns
                WHERE table_name = 'bay_locks' AND table_schema = 'public'
                ORDER BY ordinal_position
            """)
            columns = cur.fetchall()
            print('📊 bay_locks structure:')
            for col in columns:
                print(f'   {col[0]}: {col[1]} (default: {col[2]}, nullable: {col[3]})')

            # Also check if there are other columns we might have missed
            cur.execute("""
                SELECT *
                FROM bay_locks
                LIMIT 1
            """)
            try:
                sample_row = cur.fetchone()
                if sample_row:
                    print(f'📋 Sample row: {sample_row}')
                    print(f'   Column count: {len(sample_row)}')
                else:
                    print('📋 Table is empty')
            except Exception as e:
                print(f'⚠️  Error getting sample row: {e}')

            # Check current content
            cur.execute('SELECT COUNT(*) FROM bay_locks')
            count = cur.fetchone()[0]
            print(f'📈 Current bay locks: {count}')

            if count > 0:
                cur.execute('SELECT bay_id, locked_by_robot, lock_timestamp FROM bay_locks LIMIT 5')
                locks = cur.fetchall()
                for lock in locks:
                    print(f'   🔒 {lock[0]} -> {lock[1]} at {lock[2]}')

    conn.close()
    print('✅ Database connection test completed')

except Exception as e:
    print(f'❌ Database error: {e}')
    import traceback
    traceback.print_exc()
