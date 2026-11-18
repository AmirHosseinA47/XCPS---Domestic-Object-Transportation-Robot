#!/usr/bin/env python3
"""Builds the 2D occupancy map of the IAI kitchen for Nav2 from the lab's kitchen model.

Input: config/kitchen_footprints.json - the axis-aligned box of every kitchen link, measured
with the lab's own physics engine (pybullet) on the lab's kitchen URDF.
Output: maps/kitchen.pgm and maps/kitchen.yaml (Nav2 map_server format).

    python3 src/kitchen_gazebo/scripts/make_kitchen_map.py
"""
import json
import pathlib

import numpy as np

PACKAGE = pathlib.Path(__file__).resolve().parents[1]
FOOTPRINTS = PACKAGE / 'config' / 'kitchen_footprints.json'
MAP_DIR = PACKAGE / 'maps'

RESOLUTION = 0.05             # m per cell
ROBOT_HEIGHT_RANGE = (0.02, 2.0)  # an object blocks the robot if it overlaps these heights
ROOM = {'x': (-3.95, 1.95), 'y': (-2.03, 3.15)}  # outer face of the kitchen walls
MARGIN = 0.25                 # map border around the room, m
FREE, OCCUPIED = 254, 0


def cell_index(value, origin):
    return int(np.floor((value - origin) / RESOLUTION))


def main():
    boxes = json.loads(FOOTPRINTS.read_text())
    x0, x1 = ROOM['x'][0] - MARGIN, ROOM['x'][1] + MARGIN
    y0, y1 = ROOM['y'][0] - MARGIN, ROOM['y'][1] + MARGIN
    width, height = cell_index(x1, x0), cell_index(y1, y0)
    grid = np.full((height, width), OCCUPIED, dtype=np.uint8)  # rows = y, cols = x

    # Inside the walls is free, outside is not reachable.
    grid[cell_index(ROOM['y'][0], y0):cell_index(ROOM['y'][1], y0),
         cell_index(ROOM['x'][0], x0):cell_index(ROOM['x'][1], x0)] = FREE

    blocking = [b for b in boxes
                if b['max'][2] > ROBOT_HEIGHT_RANGE[0] and b['min'][2] < ROBOT_HEIGHT_RANGE[1]]
    for b in blocking:
        r0, r1 = cell_index(b['min'][1], y0), cell_index(b['max'][1], y0) + 1
        c0, c1 = cell_index(b['min'][0], x0), cell_index(b['max'][0], x0) + 1
        grid[max(r0, 0):r1, max(c0, 0):c1] = OCCUPIED

    MAP_DIR.mkdir(exist_ok=True)
    image = np.flipud(grid)  # PGM row 0 is the top (largest y)
    with open(MAP_DIR / 'kitchen.pgm', 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(image.tobytes())
    (MAP_DIR / 'kitchen.yaml').write_text(
        'image: kitchen.pgm\n'
        'mode: trinary\n'
        f'resolution: {RESOLUTION}\n'
        f'origin: [{x0:.3f}, {y0:.3f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n')
    print(f'map {width}x{height} cells ({width * RESOLUTION:.2f} x {height * RESOLUTION:.2f} m), '
          f'{len(blocking)} blocking kitchen parts, origin ({x0:.2f}, {y0:.2f})')


if __name__ == '__main__':
    main()
