#!/usr/bin/env python3
"""
Image Viewer for Lane Generator

This script displays the floorplan and generated visualization images.
"""

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os

def view_floorplan():
    """Display the original floorplan"""
    if os.path.exists('sample_floorplan.png'):
        print("Displaying original floorplan...")
        img = mpimg.imread('sample_floorplan.png')
        
        plt.figure(figsize=(10, 8))
        plt.imshow(img)
        plt.title('Original Warehouse Floorplan', fontsize=14, fontweight='bold')
        plt.axis('off')
        plt.show()
    else:
        print("Floorplan not found. Run test_lane_generator.py first.")

def view_visualization():
    """Display the generated lane visualization"""
    if os.path.exists('simple_lane_visualization.png'):
        print("Displaying lane visualization...")
        img = mpimg.imread('simple_lane_visualization.png')
        
        plt.figure(figsize=(12, 10))
        plt.imshow(img)
        plt.title('Generated Lane Navigation System', fontsize=14, fontweight='bold')
        plt.axis('off')
        plt.show()
    else:
        print("Visualization not found. Run simple_lane_visualizer.py first.")

def view_both():
    """Display both images side by side"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
    
    # Original floorplan
    if os.path.exists('sample_floorplan.png'):
        img1 = mpimg.imread('sample_floorplan.png')
        ax1.imshow(img1)
        ax1.set_title('Original Floorplan', fontsize=12, fontweight='bold')
        ax1.axis('off')
    else:
        ax1.text(0.5, 0.5, 'Floorplan not found', ha='center', va='center', transform=ax1.transAxes)
        ax1.set_title('Original Floorplan (Missing)', fontsize=12)
    
    # Generated visualization
    if os.path.exists('simple_lane_visualization.png'):
        img2 = mpimg.imread('simple_lane_visualization.png')
        ax2.imshow(img2)
        ax2.set_title('Generated Lane System', fontsize=12, fontweight='bold')
        ax2.axis('off')
    else:
        ax2.text(0.5, 0.5, 'Visualization not found', ha='center', va='center', transform=ax2.transAxes)
        ax2.set_title('Generated Lane System (Missing)', fontsize=12)
    
    plt.tight_layout()
    plt.show()

def main():
    """Main function with menu"""
    print("=" * 50)
    print("LANE GENERATOR IMAGE VIEWER")
    print("=" * 50)
    print("1. View original floorplan")
    print("2. View generated visualization")
    print("3. View both side by side")
    print("4. Exit")
    
    while True:
        choice = input("\nEnter your choice (1-4): ").strip()
        
        if choice == '1':
            view_floorplan()
        elif choice == '2':
            view_visualization()
        elif choice == '3':
            view_both()
        elif choice == '4':
            print("Goodbye!")
            break
        else:
            print("Invalid choice. Please enter 1-4.")

if __name__ == '__main__':
    main() 