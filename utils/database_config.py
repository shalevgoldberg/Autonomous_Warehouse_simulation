"""
Database Configuration Utility

This module provides a centralized, reusable function for parsing database configuration
from environment variables or provided parameters. It supports both modern DATABASE_URL
format and individual environment variables for maximum flexibility.

Design Principles:
- **Single Source of Truth**: One function handles all database configuration
- **Environment-First**: Prioritizes environment variables over hardcoded defaults
- **Flexible**: Supports multiple configuration methods
- **Error-Friendly**: Provides clear, actionable error messages
- **Type-Safe**: Returns typed configuration dictionary

Configuration Priority:
1. DATABASE_URL environment variable (modern standard)
2. Individual WAREHOUSE_DB_* environment variables
3. Provided parameters with fallbacks to defaults

Example Usage:
    from utils.database_config import get_database_config
    
    # Use with defaults
    config = get_database_config()
    
    # Use with custom fallbacks
    config = get_database_config(
        db_host="custom-host",
        db_port=5433,
        db_name="custom_db"
    )
"""
import os
import logging
from typing import Dict, Any, Optional
from urllib.parse import urlparse

# Configure logging
logger = logging.getLogger(__name__)


class DatabaseConfigError(Exception):
    """Exception raised when database configuration is invalid or incomplete."""
    pass


def get_database_config(
    db_host: str = "localhost",
    db_port: int = 5432,
    db_name: str = "warehouse_sim",
    db_user: str = "postgres",
    db_password: Optional[str] = None
) -> Dict[str, Any]:
    """
    Get database configuration from environment variables or provided parameters.

    This function implements a flexible configuration system that supports multiple
    ways to specify database connection parameters, with clear priority ordering
    and comprehensive error handling.

    Args:
        db_host: Database host (fallback if not in environment)
        db_port: Database port (fallback if not in environment)
        db_name: Database name (fallback if not in environment)
        db_user: Database user (fallback if not in environment)
        db_password: Database password (fallback if not in environment)

    Returns:
        Dictionary with database connection parameters:
        {
            'host': str,
            'port': int,
            'database': str,
            'user': str,
            'password': str
        }

    Raises:
        DatabaseConfigError: If database configuration is incomplete or invalid

    Examples:
        # Using DATABASE_URL (recommended)
        os.environ['DATABASE_URL'] = 'postgresql://user:pass@host:5432/db'
        config = get_database_config()

        # Using individual environment variables
        os.environ['WAREHOUSE_DB_HOST'] = 'localhost'
        os.environ['WAREHOUSE_DB_PASSWORD'] = 'mypassword'
        config = get_database_config()

        # Using parameters with fallbacks
        config = get_database_config(
            db_host='custom-host',
            db_password='custom-password'
        )
    """
    # Priority 1: Check for DATABASE_URL (modern standard)
    database_url = os.getenv('DATABASE_URL')
    if database_url:
        try:
            parsed = urlparse(database_url)
            if not parsed.scheme or not parsed.hostname:
                raise ValueError("Invalid DATABASE_URL format - missing scheme or hostname")

            config = {
                'host': parsed.hostname,
                'port': parsed.port or 5432,
                'database': parsed.path.lstrip('/') or 'warehouse_sim',
                'user': parsed.username or 'postgres',
                'password': parsed.password
            }

            # Validate required fields
            if not all([config['host'], config['database'], config['user']]):
                raise ValueError("DATABASE_URL missing required components (host, database, user)")

            logger.info(f"Using DATABASE_URL configuration: {parsed.hostname}:{parsed.port}/{parsed.path.lstrip('/')}")
            return config

        except Exception as e:
            logger.warning(f"Invalid DATABASE_URL format: {e}. Falling back to individual variables.")
            # Continue to fallback method

    # Priority 2: Fall back to individual environment variables
    env_config = {
        'host': os.getenv('WAREHOUSE_DB_HOST', db_host),
        'port': int(os.getenv('WAREHOUSE_DB_PORT', str(db_port))),
        'database': os.getenv('WAREHOUSE_DB_NAME', db_name),
        'user': os.getenv('WAREHOUSE_DB_USER', db_user),
        'password': os.getenv('WAREHOUSE_DB_PASSWORD', db_password)
    }

    # If password still not set, try legacy environment variable
    if not env_config['password']:
        env_config['password'] = os.getenv('WAREHOUSE_DB_PASSWORD')

    # Priority 3: Validate configuration completeness
    if not env_config['password']:
        raise DatabaseConfigError(
            "Database password not configured. Please set one of:\n"
            "1. DATABASE_URL=\"postgresql://user:password@host:port/database\"\n"
            "2. WAREHOUSE_DB_PASSWORD environment variable\n"
            "3. Pass db_password parameter\n\n"
            "Examples:\n"
            "Windows: $env:DATABASE_URL=\"postgresql://postgres:mypassword@localhost:5432/warehouse_sim\"\n"
            "macOS/Linux: export DATABASE_URL=\"postgresql://postgres:mypassword@localhost:5432/warehouse_sim\"\n"
            "Individual: export WAREHOUSE_DB_PASSWORD=\"mypassword\""
        )

    logger.info(f"Using individual environment variables: {env_config['host']}:{env_config['port']}/{env_config['database']}")
    return env_config


def validate_database_config(config: Dict[str, Any]) -> bool:
    """
    Validate that a database configuration dictionary contains all required fields.

    Args:
        config: Database configuration dictionary

    Returns:
        True if configuration is valid, False otherwise

    Example:
        config = get_database_config()
        if validate_database_config(config):
            print("Configuration is valid")
    """
    required_fields = ['host', 'port', 'database', 'user', 'password']
    return all(field in config and config[field] for field in required_fields)


def get_database_connection_string(config: Dict[str, Any]) -> str:
    """
    Convert database configuration dictionary to PostgreSQL connection string.

    Args:
        config: Database configuration dictionary

    Returns:
        PostgreSQL connection string

    Example:
        config = get_database_config()
        conn_str = get_database_connection_string(config)
        # Returns: "host=localhost port=5432 dbname=warehouse_sim user=postgres password=***"
    """
    return (
        f"host={config['host']} "
        f"port={config['port']} "
        f"dbname={config['database']} "
        f"user={config['user']} "
        f"password={config['password']}"
    )
