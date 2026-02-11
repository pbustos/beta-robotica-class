#!/usr/bin/env python3
"""
Test MRPT Map Generator
Creates sample .gridmap files for testing the MRPT loader functionality
"""

import struct
import argparse
import math
import os

def create_simple_grid_map(width, height, resolution, origin_x, origin_y, output_file):
    """
    Create a simple rectangular room map in MRPT binary format

    Args:
        width: Number of cells in X
        height: Number of cells in Y
        resolution: mm per cell
        origin_x: Origin X in meters
        origin_y: Origin Y in meters
        output_file: Path to save the .gridmap file
    """
    print(f"Creating {width}x{height} map at {resolution}mm resolution...")

    # Create grid data
    grid = []
    for y in range(height):
        for x in range(width):
            # Create a simple room with walls
            if x < 2 or x >= width-2 or y < 2 or y >= height-2:
                # Walls (occupied)
                grid.append(0.95)
            elif 30 < x < 40 and 30 < y < 40:
                # Obstacle in the center
                grid.append(0.9)
            else:
                # Free space
                grid.append(0.1)

    # Write binary format
    with open(output_file, 'wb') as f:
        # Header
        f.write(struct.pack('<I', width))      # uint32 width
        f.write(struct.pack('<I', height))     # uint32 height
        f.write(struct.pack('<f', float(resolution)))  # float resolution
        f.write(struct.pack('<f', origin_x))   # float origin_x (meters)
        f.write(struct.pack('<f', origin_y))   # float origin_y (meters)

        # Grid data
        for occupancy in grid:
            f.write(struct.pack('<f', occupancy))

    print(f"✓ Created: {output_file}")

def create_circular_obstacle_map(width, height, resolution, origin_x, origin_y, output_file):
    """
    Create a map with circular obstacles
    """
    print(f"Creating circular obstacle map ({width}x{height})...")

    grid = []
    center_x = width / 2
    center_y = height / 2

    for y in range(height):
        for x in range(width):
            # Walls
            if x < 2 or x >= width-2 or y < 2 or y >= height-2:
                grid.append(0.95)
            else:
                # Multiple circular obstacles
                dist1 = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                dist2 = math.sqrt((x - 20)**2 + (y - 20)**2)
                dist3 = math.sqrt((x - width+20)**2 + (y - height+20)**2)

                if dist1 < 8 or dist2 < 5 or dist3 < 5:
                    grid.append(0.9)
                else:
                    grid.append(0.1)

    with open(output_file, 'wb') as f:
        f.write(struct.pack('<I', width))
        f.write(struct.pack('<I', height))
        f.write(struct.pack('<f', float(resolution)))
        f.write(struct.pack('<f', origin_x))
        f.write(struct.pack('<f', origin_y))
        for occupancy in grid:
            f.write(struct.pack('<f', occupancy))

    print(f"✓ Created: {output_file}")

def create_yaml_format_map(width, height, resolution, origin_x, origin_y, output_file):
    """
    Create a map in YAML format
    """
    print(f"Creating YAML format map ({width}x{height})...")

    grid = []
    for y in range(height):
        for x in range(width):
            if x < 2 or x >= width-2 or y < 2 or y >= height-2:
                grid.append(0.95)
            elif 25 < x < 35 and 25 < y < 35:
                grid.append(0.85)
            else:
                grid.append(0.05)

    with open(output_file, 'w') as f:
        f.write(f"# MRPT OccupancyGrid Map (YAML format)\n")
        f.write(f"width: {width}\n")
        f.write(f"height: {height}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin_x: {origin_x}\n")
        f.write(f"origin_y: {origin_y}\n")
        f.write(f"data: |\n")

        # Write grid data in rows
        for y in range(height):
            row = []
            for x in range(width):
                row.append(f"{grid[y*width + x]:.2f}")
            f.write("  " + " ".join(row) + "\n")

    print(f"✓ Created: {output_file}")

def main():
    parser = argparse.ArgumentParser(description="Generate test MRPT gridmap files")
    parser.add_argument("--output-dir", default="./test_maps", help="Output directory")
    parser.add_argument("--size", default="small", choices=["small", "medium", "large"],
                       help="Map size")

    args = parser.parse_args()

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    # Define sizes
    sizes = {
        "small": (50, 50, 100, -2.5, -2.5),
        "medium": (100, 100, 100, -5.0, -5.0),
        "large": (200, 200, 50, -5.0, -5.0)
    }

    width, height, resolution, orig_x, orig_y = sizes[args.size]

    # Generate test maps
    create_simple_grid_map(
        width, height, resolution, orig_x, orig_y,
        os.path.join(args.output_dir, "simple_room.gridmap")
    )

    create_circular_obstacle_map(
        width, height, resolution, orig_x, orig_y,
        os.path.join(args.output_dir, "circular_obstacles.gridmap")
    )

    create_yaml_format_map(
        width, height, resolution, orig_x, orig_y,
        os.path.join(args.output_dir, "yaml_format.gridmap")
    )

    print(f"\n✓ Generated {args.size} test maps in: {args.output_dir}")
    print("\nUsage in Gridder component:")
    print(f"  Gridder_loadMRPTMap(\"{args.output_dir}/simple_room.gridmap\")")

if __name__ == "__main__":
    main()
