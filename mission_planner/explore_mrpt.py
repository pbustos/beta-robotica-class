#!/usr/bin/env python3
import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')

with open('/tmp/mrpt_explore.log', 'w') as f:
    try:
        from mrpt.pymrpt import mrpt
        f.write("pymrpt OK\n")
        f.write(f"serialization: {dir(mrpt.serialization)}\n\n")
        f.write(f"io: {dir(mrpt.io)}\n\n")
        f.write(f"maps: {dir(mrpt.maps)}\n\n")

        # Check COccupancyGridMap2D methods
        gm = mrpt.maps.COccupancyGridMap2D()
        f.write(f"COccupancyGridMap2D methods: {[m for m in dir(gm) if not m.startswith('_')]}\n")

    except Exception as e:
        import traceback
        f.write(f"Error: {e}\n")
        f.write(traceback.format_exc())
