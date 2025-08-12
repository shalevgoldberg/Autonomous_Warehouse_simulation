#!/usr/bin/env python3
"""
Minimal test to identify the hanging issue
"""

import sys
import os
import time
import logging
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def test_thread_pool_timeout():
    """Test if ThreadPoolExecutor with timeout is working correctly"""
    logger.info("Testing ThreadPoolExecutor timeout behavior")
    
    def worker_function(worker_id):
        logger.info(f"Worker {worker_id} starting")
        time.sleep(2)  # Simulate work
        logger.info(f"Worker {worker_id} completed")
        return f"result_{worker_id}"
    
    start_time = time.time()
    
    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            logger.info("Submitting tasks")
            futures = [executor.submit(worker_function, i) for i in range(3)]
            
            logger.info("Waiting for results with timeout")
            results = []
            for future in as_completed(futures, timeout=1.0):  # 1 second timeout
                try:
                    result = future.result(timeout=0.5)
                    results.append(result)
                    logger.info(f"Got result: {result}")
                except Exception as e:
                    logger.error(f"Error getting result: {e}")
            
            logger.info(f"Collected {len(results)} results")
    
    except Exception as e:
        logger.error(f"ThreadPoolExecutor test failed: {e}")
    
    end_time = time.time()
    logger.info(f"Test completed in {end_time - start_time:.3f} seconds")

if __name__ == "__main__":
    logger.info("Starting minimal test")
    test_thread_pool_timeout()
    logger.info("Test completed") 