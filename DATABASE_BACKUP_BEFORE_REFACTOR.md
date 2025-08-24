# Database Backup Documentation - Before Conflict Box Queue Refactoring

**Date**: $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")
**Purpose**: Backup of current database structure before implementing the new conflict box queue architecture
**Database**: warehouse_sim
**Version**: Pre-refactor state

## 🚨 IMPORTANT: DO NOT DELETE THIS FILE

This document contains the complete backup of the current database structure. If anything goes wrong during the refactoring, use this to restore the database to its working state.

## 📊 Database Overview

The database contains 35 relations including:
- **Core Tables**: conflict_boxes, conflict_box_locks, conflict_box_queue, conflict_box_queue_stats
- **Navigation Tables**: lanes, navigation_graph_nodes, navigation_graph_edges
- **Robot Tables**: robots, robot_metrics
- **Inventory Tables**: shelves, shelf_inventory, items
- **Simulation Tables**: simulation_metrics, tasks, raw_orders

## 🎯 Conflict Box Related Tables (CRITICAL)

### 1. conflict_boxes
**Purpose**: Store conflict box definitions for intersection management

```sql
CREATE TABLE conflict_boxes (
    box_id character varying(36) NOT NULL,
    center_x numeric(10,3) NOT NULL,
    center_y numeric(10,3) NOT NULL,
    size numeric(5,3) NOT NULL,
    created_at timestamp without time zone DEFAULT CURRENT_TIMESTAMP,
    updated_at timestamp without time zone DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT conflict_boxes_pkey PRIMARY KEY (box_id),
    CONSTRAINT conflict_boxes_size_check CHECK (size > 0::numeric)
);
```

**Indexes**:
- `conflict_boxes_pkey` PRIMARY KEY, btree (box_id)
- `idx_conflict_boxes_position` btree (center_x, center_y)

**Foreign Key References**:
- `conflict_box_locks.box_id` → `conflict_boxes.box_id`
- `conflict_box_queue_stats.box_id` → `conflict_boxes.box_id`
- `navigation_graph_nodes.conflict_box_id` → `conflict_boxes.box_id`

### 2. conflict_box_locks
**Purpose**: Manage active locks on conflict boxes with heartbeat mechanism

```sql
CREATE TABLE conflict_box_locks (
    box_id character varying(36) NOT NULL,
    locked_by_robot character varying(36) NOT NULL,
    priority integer DEFAULT 0,
    locked_at timestamp without time zone DEFAULT CURRENT_TIMESTAMP,
    heartbeat_at timestamp without time zone DEFAULT CURRENT_TIMESTAMP,
    lock_timeout_seconds integer DEFAULT 30,
    robot_inside boolean DEFAULT false,
    CONSTRAINT conflict_box_locks_pkey PRIMARY KEY (box_id),
    CONSTRAINT conflict_box_locks_box_id_fkey FOREIGN KEY (box_id) REFERENCES conflict_boxes(box_id),
    CONSTRAINT conflict_box_locks_locked_by_robot_fkey FOREIGN KEY (locked_by_robot) REFERENCES robots(robot_id)
);
```

**Indexes**:
- `conflict_box_locks_pkey` PRIMARY KEY, btree (box_id)
- `idx_conflict_box_locks_heartbeat` btree (heartbeat_at)
- `idx_conflict_box_locks_priority` btree (priority)
- `idx_conflict_box_locks_robot` btree (locked_by_robot)

### 3. conflict_box_queue
**Purpose**: Manage robot queue for conflict box access with priority-based ordering

```sql
CREATE TABLE conflict_box_queue (
    id integer NOT NULL DEFAULT nextval('conflict_box_queue_id_seq'::regclass),
    box_id character varying(50) NOT NULL,
    robot_id character varying(50) NOT NULL,
    priority integer NOT NULL DEFAULT 0,
    queue_time timestamp without time zone NOT NULL DEFAULT CURRENT_TIMESTAMP,
    timeout_time timestamp without time zone NOT NULL,
    position character varying(20) NOT NULL DEFAULT 'queued'::character varying,
    estimated_wait_time real DEFAULT 0.0,
    created_at timestamp without time zone DEFAULT CURRENT_TIMESTAMP,
    updated_at timestamp without time zone DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT conflict_box_queue_pkey PRIMARY KEY (id),
    CONSTRAINT conflict_box_queue_box_id_robot_id_key UNIQUE (box_id, robot_id)
);
```

**Indexes**:
- `conflict_box_queue_pkey` PRIMARY KEY, btree (id)
- `conflict_box_queue_box_id_robot_id_key` UNIQUE CONSTRAINT, btree (box_id, robot_id)
- `idx_conflict_box_queue_box_id` btree (box_id)
- `idx_conflict_box_queue_box_priority` btree (box_id, priority DESC)
- `idx_conflict_box_queue_priority_time` btree (box_id, priority DESC, queue_time)
- `idx_conflict_box_queue_robot_id` btree (robot_id)
- `idx_conflict_box_queue_timeout` btree (timeout_time)

**Sequence**:
- `conflict_box_queue_id_seq` (owned by conflict_box_queue.id)

### 4. conflict_box_queue_stats
**Purpose**: Collect historical statistics for conflict box usage and performance

```sql
CREATE TABLE conflict_box_queue_stats (
    box_id character varying(36) NOT NULL,
    total_requests bigint DEFAULT 0,
    successful_acquisitions bigint DEFAULT 0,
    timeouts bigint DEFAULT 0,
    cancellations bigint DEFAULT 0,
    total_wait_time numeric(10,2) DEFAULT 0.0,
    total_lock_time numeric(10,2) DEFAULT 0.0,
    last_updated timestamp without time zone DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT conflict_box_queue_stats_pkey PRIMARY KEY (box_id),
    CONSTRAINT conflict_box_queue_stats_box_id_fkey FOREIGN KEY (box_id) REFERENCES conflict_boxes(box_id)
);
```

**Indexes**:
- `conflict_box_queue_stats_pkey` PRIMARY KEY, btree (box_id)

## 🔧 Database Functions (CRITICAL)

### 1. get_queue_position
**Purpose**: Calculate robot's position in queue based on priority and queue time

```sql
CREATE OR REPLACE FUNCTION get_queue_position(p_box_id character varying, p_robot_id character varying)
RETURNS integer
LANGUAGE plpgsql
AS $function$
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
$function$;
```

**Status**: ✅ WORKING - This function is functional and correctly calculates queue positions

### 2. update_queue_positions
**Purpose**: Update queue positions using window functions (currently broken due to PostgreSQL limitations)

```sql
CREATE OR REPLACE FUNCTION update_queue_positions(p_box_id character varying)
RETURNS void
LANGUAGE plpgsql
AS $function$
BEGIN
    UPDATE conflict_box_queue
    SET position = CASE
        WHEN ROW_NUMBER() OVER (ORDER BY priority DESC, queue_time ASC) = 1 THEN 'next_in_line'
        ELSE 'queued'
    END,
    updated_at = CURRENT_TIMESTAMP
    WHERE box_id = p_box_id AND position != 'lock_acquired';
END;
$function$;
```

**Status**: ❌ BROKEN - This function uses window functions in UPDATE statements, which PostgreSQL doesn't allow

### 3. update_queue_positions_simple
**Purpose**: Placeholder function that does nothing (temporary workaround)

```sql
CREATE OR REPLACE FUNCTION update_queue_positions_simple(p_box_id character varying)
RETURNS void
LANGUAGE sql
AS $function$
SELECT NULL;
$function$;
```

**Status**: ⚠️ PLACEHOLDER - This function is intentionally a no-op as a temporary workaround

## 📋 Current Data Sample

### Sample Conflict Box Data
```sql
-- Check current conflict boxes
SELECT box_id, center_x, center_y, size, created_at 
FROM conflict_boxes 
LIMIT 5;
```

### Sample Queue Data
```sql
-- Check current queue entries
SELECT box_id, robot_id, priority, position, queue_time, timeout_time
FROM conflict_box_queue 
WHERE position IN ('queued', 'next_in_line', 'lock_acquired')
LIMIT 10;
```

### Sample Lock Data
```sql
-- Check current active locks
SELECT box_id, locked_by_robot, priority, locked_at, heartbeat_at
FROM conflict_box_locks
LIMIT 5;
```

## 🔍 Current Issues Identified

### 1. Broken Queue Position Updates
- **Problem**: `update_queue_positions()` function uses window functions in UPDATE statements
- **Impact**: Queue positions are not properly maintained
- **Workaround**: Currently using `update_queue_positions_simple()` which does nothing

### 2. Missing Position Reordering Logic
- **Problem**: When robots release locks, queue positions are not recalculated
- **Impact**: Robots may get stuck in incorrect queue positions
- **Current State**: Manual position updates are attempted but fail

### 3. Inconsistent State Management
- **Problem**: Queue state and lock state can become out of sync
- **Impact**: Robots may think they have locks when they don't
- **Current State**: Basic validation exists but is not comprehensive

## 🛡️ Backup and Recovery Procedures

### 1. Database Backup (Before Refactoring)
```bash
# Create full database backup
pg_dump -h localhost -U postgres -d warehouse_sim > warehouse_sim_backup_$(date +%Y%m%d_%H%M%S).sql

# Create schema-only backup
pg_dump -h localhost -U postgres -d warehouse_sim --schema-only > warehouse_sim_schema_backup_$(date +%Y%m%d_%H%M%S).sql
```

### 2. Function Backup
```sql
-- Backup current function definitions
SELECT 
    'CREATE OR REPLACE FUNCTION ' || 
    p.proname || '(' || 
    pg_get_function_arguments(p.oid) || ') ' ||
    'RETURNS ' || pg_get_function_result(p.oid) || ' ' ||
    'LANGUAGE ' || l.lanname || ' ' ||
    'AS $function$' || chr(10) ||
    pg_get_functiondef(p.oid) || chr(10) ||
    '$function$;' as backup_script
FROM pg_proc p
JOIN pg_namespace n ON p.pronamespace = n.oid
JOIN pg_language l ON p.prolang = l.oid
WHERE n.nspname = 'public' 
AND p.proname IN ('get_queue_position', 'update_queue_positions', 'update_queue_positions_simple');
```

### 3. Table Structure Backup
```sql
-- Backup table creation scripts
SELECT 
    'CREATE TABLE ' || tablename || ' (' || chr(10) ||
    string_agg(
        column_name || ' ' || data_type || 
        CASE WHEN character_maximum_length IS NOT NULL 
             THEN '(' || character_maximum_length || ')' 
             ELSE '' 
        END ||
        CASE WHEN is_nullable = 'NO' THEN ' NOT NULL' ELSE '' END ||
        CASE WHEN column_default IS NOT NULL 
             THEN ' DEFAULT ' || column_default 
             ELSE '' 
        END,
        ',' || chr(10)
        ORDER BY ordinal_position
    ) || chr(10) || ');' as create_table_script
FROM information_schema.columns
WHERE table_schema = 'public' 
AND table_name IN ('conflict_boxes', 'conflict_box_locks', 'conflict_box_queue', 'conflict_box_queue_stats')
GROUP BY tablename;
```

### 4. Index Backup
```sql
-- Backup index creation scripts
SELECT 
    'CREATE INDEX ' || indexname || ' ON ' || tablename || ' (' ||
    string_agg(attname, ', ' ORDER BY attnum) || ');' as create_index_script
FROM pg_index i
JOIN pg_class c ON i.indexrelid = c.oid
JOIN pg_class t ON i.indrelid = t.oid
JOIN pg_attribute a ON a.attrelid = t.oid AND a.attnum = ANY(i.indkey)
WHERE t.relname IN ('conflict_boxes', 'conflict_box_locks', 'conflict_box_queue', 'conflict_box_queue_stats')
AND c.relname NOT LIKE '%_pkey'
GROUP BY indexname, tablename;
```

## 🚀 Refactoring Plan

### Phase 1: Database Functions (Day 1)
1. **Fix `get_queue_position`**: Optimize with window functions
2. **Replace `update_queue_positions`**: Implement working position update logic
3. **Remove `update_queue_positions_simple`**: No longer needed

### Phase 2: Service Implementation (Day 2-3)
1. **Implement `ConflictBoxQueueManager`**: Core queue logic
2. **Implement `ConflictBoxLockManager`**: Lock coordination
3. **Implement `ConflictBoxStatistics`**: Metrics collection

### Phase 3: Integration (Day 4-5)
1. **Update `ConflictBoxQueueImpl`**: Use new service architecture
2. **Test with real robots**: Verify end-to-end functionality
3. **Performance validation**: Ensure no regression

## ⚠️ Rollback Procedures

### If Functions Break
```sql
-- Restore original function definitions
-- (Use the backup scripts above)
```

### If Tables Get Corrupted
```sql
-- Restore from schema backup
-- (Use the schema backup created above)
```

### If Data Gets Lost
```sql
-- Restore from full database backup
-- (Use the full backup created above)
```

## 📞 Emergency Contacts

- **Database Admin**: System administrator
- **Backup Location**: This document and generated backup files
- **Recovery Time**: Estimated 15-30 minutes for full restore

## ✅ Pre-Refactoring Checklist

- [ ] Full database backup completed
- [ ] Schema backup completed
- [ ] Function backup completed
- [ ] Index backup completed
- [ ] Current data sample documented
- [ ] Rollback procedures tested
- [ ] Team notified of maintenance window

---

**⚠️ REMEMBER**: This document is your lifeline if something goes wrong. Keep it safe and accessible during the entire refactoring process.

**📅 Next Review**: After refactoring completion
**🔧 Maintained By**: Development Team
**📋 Version**: 1.0 (Pre-refactor state)

