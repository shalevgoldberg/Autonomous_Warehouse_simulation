"""
Robot Registry Implementation

Provides a professional, SOLID-compliant implementation of IRobotRegistry
that uses the existing psycopg2 connection pool from SimulationDataServiceImpl
to manage robot identity and runtime state persistence. All fallbacks and
unexpected conditions are logged with context.
"""
from __future__ import annotations

import logging
from typing import List, Optional, Tuple

from interfaces.robot_registry_interface import (
    IRobotRegistry,
    RobotIdentity,
    RobotRuntimeState,
)
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
import uuid


logger = logging.getLogger(__name__)


class RobotRegistryImpl(IRobotRegistry):
    """
    PostgreSQL-backed robot registry leveraging the existing connection pool
    from SimulationDataServiceImpl to avoid duplicate pool management.
    """

    def __init__(self, simulation_data_service: SimulationDataServiceImpl):
        self._sds = simulation_data_service

    def list_registered_robots(self) -> List[RobotIdentity]:
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        SELECT robot_id, COALESCE(name, '') AS name
                        FROM robots
                        ORDER BY name ASC, robot_id ASC
                        """
                    )
                    rows = cur.fetchall()
                    return [RobotIdentity(robot_id=r['robot_id'], name=r['name'] or None) for r in rows]
        except Exception as e:
            logger.error("RobotRegistry.list_registered_robots failed: %s", e, exc_info=True)
            return []

    def get_robot_state(self, robot_id: str) -> Optional[RobotRuntimeState]:
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        SELECT pos_x, pos_y, theta, battery_level
                        FROM robot_runtime_state
                        WHERE robot_id = %s
                        """,
                        (robot_id,),
                    )
                    row = cur.fetchone()
                    if not row:
                        return None
                    pos_x, pos_y, theta, battery = row['pos_x'], row['pos_y'], row['theta'], row['battery_level']
                    position: Optional[Tuple[float, float, float]]
                    if pos_x is None or pos_y is None or theta is None:
                        position = None
                    else:
                        position = (float(pos_x), float(pos_y), float(theta))
                    return RobotRuntimeState(robot_id=robot_id, position=position, battery_level=float(battery))
        except Exception as e:
            logger.error("RobotRegistry.get_robot_state failed for %s: %s", robot_id, e, exc_info=True)
            return None

    def upsert_robot_state(self, robot_id: str, position: Optional[Tuple[float, float, float]],
                           battery_level: float) -> None:
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    if position is None:
                        cur.execute(
                            """
                            INSERT INTO robot_runtime_state (robot_id, pos_x, pos_y, theta, battery_level)
                            VALUES (%s, NULL, NULL, NULL, %s)
                            ON CONFLICT (robot_id)
                            DO UPDATE SET pos_x = EXCLUDED.pos_x,
                                          pos_y = EXCLUDED.pos_y,
                                          theta = EXCLUDED.theta,
                                          battery_level = EXCLUDED.battery_level,
                                          updated_at = CURRENT_TIMESTAMP
                            """,
                            (robot_id, battery_level),
                        )
                    else:
                        x, y, th = position
                        cur.execute(
                            """
                            INSERT INTO robot_runtime_state (robot_id, pos_x, pos_y, theta, battery_level)
                            VALUES (%s, %s, %s, %s, %s)
                            ON CONFLICT (robot_id)
                            DO UPDATE SET pos_x = EXCLUDED.pos_x,
                                          pos_y = EXCLUDED.pos_y,
                                          theta = EXCLUDED.theta,
                                          battery_level = EXCLUDED.battery_level,
                                          updated_at = CURRENT_TIMESTAMP
                            """,
                            (robot_id, x, y, th, battery_level),
                        )
                    conn.commit()
        except Exception as e:
            logger.error("RobotRegistry.upsert_robot_state failed for %s: %s", robot_id, e, exc_info=True)
            # Do not raise to avoid cascading failures; caller may retry

    def clear_robot_state(self, robot_id: str) -> None:
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    # Ensure row exists with default battery, then clear position only
                    cur.execute(
                        """
                        INSERT INTO robot_runtime_state (robot_id, pos_x, pos_y, theta, battery_level)
                        VALUES (%s, NULL, NULL, NULL, 1.0)
                        ON CONFLICT (robot_id)
                        DO UPDATE SET pos_x = NULL, pos_y = NULL, theta = NULL, updated_at = CURRENT_TIMESTAMP
                        """,
                        (robot_id,),
                    )
                    conn.commit()
        except Exception as e:
            logger.error("RobotRegistry.clear_robot_state failed for %s: %s", robot_id, e, exc_info=True)

    def clear_all_positions(self) -> int:
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    # Ensure a row exists per robot, then nullify positions in bulk
                    cur.execute(
                        """
                        INSERT INTO robot_runtime_state (robot_id)
                        SELECT r.robot_id FROM robots r
                        ON CONFLICT (robot_id) DO NOTHING
                        """
                    )
                    cur.execute(
                        """
                        UPDATE robot_runtime_state
                        SET pos_x = NULL, pos_y = NULL, theta = NULL, updated_at = CURRENT_TIMESTAMP
                        """
                    )
                    affected = cur.rowcount or 0
                    conn.commit()
                    return int(affected)
        except Exception as e:
            logger.error("RobotRegistry.clear_all_positions failed: %s", e, exc_info=True)
            return 0

    def charge_all_to_full(self) -> int:
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    # Ensure rows exist for all robots
                    cur.execute(
                        """
                        INSERT INTO robot_runtime_state (robot_id)
                        SELECT r.robot_id FROM robots r
                        ON CONFLICT (robot_id) DO NOTHING
                        """
                    )
                    cur.execute(
                        """
                        UPDATE robot_runtime_state
                        SET battery_level = 1.0, updated_at = CURRENT_TIMESTAMP
                        """
                    )
                    affected = cur.rowcount or 0
                    conn.commit()
                    return int(affected)
        except Exception as e:
            logger.error("RobotRegistry.charge_all_to_full failed: %s", e, exc_info=True)
            return 0

    def register_robot(self, robot_id: str, name: Optional[str] = None) -> bool:
        """
        Register a new robot in the robots table. Returns True if created, False if already exists.
        """
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    # Generate default name if not provided
                    if name is None:
                        name = f"Robot {robot_id.replace('_', ' ').title()}"
                    
                    cur.execute(
                        """
                        INSERT INTO robots (robot_id, name)
                        VALUES (%s, %s)
                        ON CONFLICT (robot_id) DO NOTHING
                        """,
                        (robot_id, name),
                    )
                    created = cur.rowcount > 0
                    conn.commit()
                    return bool(created)
        except Exception as e:
            logger.error("RobotRegistry.register_robot failed for %s: %s", robot_id, e, exc_info=True)
            return False

    def delete_robot(self, robot_id: str) -> bool:
        """Delete robot from registry (and cascade runtime state via FK)."""
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        DELETE FROM robots WHERE robot_id = %s
                        """,
                        (robot_id,),
                    )
                    deleted = cur.rowcount > 0
                    conn.commit()
                    if not deleted:
                        logger.warning("RobotRegistry.delete_robot: robot_id %s not found", robot_id)
                    return bool(deleted)
        except Exception as e:
            logger.error("RobotRegistry.delete_robot failed for %s: %s", robot_id, e, exc_info=True)
            return False

    def delete_all_robots(self) -> int:
        """Delete all robots and cascade runtime state via FK; returns count."""
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    cur.execute("SELECT COUNT(*) AS c FROM robots")
                    before = int((cur.fetchone() or {}).get('c', 0))
                    cur.execute("DELETE FROM robots")
                    conn.commit()
                    return before
        except Exception as e:
            logger.error("RobotRegistry.delete_all_robots failed: %s", e, exc_info=True)
            return 0

    def register_robots(self, count: int, name_prefix: str = "Robot") -> List[RobotIdentity]:
        """Register 'count' robots with generated IDs and default names."""
        created: List[RobotIdentity] = []
        if count <= 0:
            return created
        try:
            with self._sds._get_connection() as conn:  # type: ignore[attr-defined]
                with conn.cursor() as cur:
                    for _ in range(count):
                        rid = uuid.uuid4().hex  # MuJoCo-safe ID without hyphens
                        name = f"{name_prefix} {rid[:8].upper()}"
                        try:
                            cur.execute(
                                """
                                INSERT INTO robots (robot_id, name)
                                VALUES (%s, %s)
                                ON CONFLICT (robot_id) DO NOTHING
                                """,
                                (rid, name),
                            )
                            if cur.rowcount > 0:
                                created.append(RobotIdentity(robot_id=rid, name=name))
                        except Exception as e:
                            logger.error("RobotRegistry.register_robots: failed to register %s: %s", rid, e, exc_info=True)
                    conn.commit()
            return created
        except Exception as e:
            logger.error("RobotRegistry.register_robots failed: %s", e, exc_info=True)
            return created


