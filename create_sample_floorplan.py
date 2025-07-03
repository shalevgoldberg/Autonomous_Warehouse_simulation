#!/usr/bin/env python3
"""
Create Sample Floorplan for Lane Generator Testing

This script creates a sample warehouse floorplan image with:
- White areas: drivable space
- Black areas: obstacles/walls
- Light grey areas: bays of different types
"""

import numpy as np
import imageio
import os

def create_sample_floorplan(width=100, height=80, output_path="sample_floorplan.png"):
    """
    Create a sample warehouse floorplan
    
    Args:
        width: Image width in pixels
        height: Image height in pixels
        output_path: Output file path
    """
    # Create RGB image (white background)
    floorplan = np.ones((height, width, 3), dtype=np.uint8) * 255
    
    # Add walls (black borders)
    floorplan[0:2, :] = 0  # Top wall
    floorplan[-2:, :] = 0  # Bottom wall
    floorplan[:, 0:2] = 0  # Left wall
    floorplan[:, -2:] = 0  # Right wall
    
    # Add internal obstacles (shelves)
    # Vertical shelves
    for x in range(20, width-20, 25):
        floorplan[15:height-15, x:x+3] = 0
    
    # Horizontal shelves
    for y in range(25, height-25, 20):
        floorplan[y:y+3, 15:width-15] = 0
    
    # Add bays (different grey levels for different types)
    bay_colors = {
        'dropoff': [200, 200, 200],   # Light grey
        'idle': [160, 160, 160],      # Medium grey
        'charging': [120, 120, 120]   # Dark grey
    }
    
    # Add some dropoff bays
    floorplan[10:15, 10:15] = bay_colors['dropoff']
    floorplan[10:15, width-15:width-10] = bay_colors['dropoff']
    
    # Add idle bays
    floorplan[height-15:height-10, 10:15] = bay_colors['idle']
    floorplan[height-15:height-10, width-15:width-10] = bay_colors['idle']
    
    # Add charging bays
    floorplan[height//2-5:height//2, 5:10] = bay_colors['charging']
    floorplan[height//2-5:height//2, width-10:width-5] = bay_colors['charging']
    
    # Ensure some clear corridors for lanes
    # Main horizontal corridor
    floorplan[height//2-3:height//2+3, 5:width-5] = 255
    
    # Main vertical corridors
    floorplan[5:height-5, width//4-3:width//4+3] = 255
    floorplan[5:height-5, 3*width//4-3:3*width//4+3] = 255
    
    # Save the floorplan
    imageio.imwrite(output_path, floorplan)
    print(f"Sample floorplan created: {output_path}")
    print(f"Dimensions: {width}x{height} pixels")
    print("Colors:")
    print("  White (255,255,255): Drivable area")
    print("  Black (0,0,0): Obstacles/walls")
    print("  Light grey (200,200,200): Dropoff bays")
    print("  Medium grey (160,160,160): Idle bays")
    print("  Dark grey (120,120,120): Charging bays")

if __name__ == '__main__':
    create_sample_floorplan() 