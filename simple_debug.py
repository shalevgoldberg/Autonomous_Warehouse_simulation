#!/usr/bin/env python3
"""
Simple debug script for parallel bidding
"""

import time
import logging
from concurrent.futures import ThreadPoolExecutor, as_completed

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def simple_bid_calculation(robot_id):
    """Simple bid calculation that should complete quickly"""
    logger.info(f"Robot {robot_id} starting bid calculation")
    time.sleep(0.1)  # Small delay to simulate work
    bid = robot_id * 10.0
    logger.info(f"Robot {robot_id} completed bid: {bid}")
    return bid

def test_simple_parallel():
    """Test simple parallel execution"""
    logger.info("Testing simple parallel execution")
    
    with ThreadPoolExecutor(max_workers=3) as executor:
        futures = [executor.submit(simple_bid_calculation, i) for i in range(3)]
        
        results = []
        for future in as_completed(futures, timeout=5.0):
            try:
                result = future.result(timeout=1.0)
                results.append(result)
                logger.info(f"Got result: {result}")
            except Exception as e:
                logger.error(f"Error: {e}")
    
    logger.info(f"All results: {results}")
    return len(results) == 3

if __name__ == "__main__":
    logger.info("Starting simple debug test")
    success = test_simple_parallel()
    logger.info(f"Test {'PASSED' if success else 'FAILED'}") 