import os
import time

import cv2
import grid_pathfinding
import numpy as np
from bresenham import bresenham

W, H = 300, 200
RADIUS = 21
START = (10, 10)
END = (3 * W // 4, round(H * 0.8))


def disk(w, h, x, y, radius):
    a, b = np.ogrid[-x:w - x, -y:h - y]
    mask = a * a + b * b <= radius * radius
    return mask


def fixed_size_disk(radius):
    return disk(radius * 2 + 1, radius * 2 + 1, radius, radius, radius)


if __name__ == "__main__":
    time_report = {}

    t = time.perf_counter()
    time_report[f"No-op"] = time.perf_counter() - t

    t = time.perf_counter()
    obs = np.zeros((W, H), dtype=np.uint8)
    time_report[f"Time to instantiate a ({W}, {H}) array"] = time.perf_counter() - t

    obs[W // 2, H // 2] = True
    obs[32 * W // 100, 4 * H // 6] = True
    obs[W // 2, H // 4] = True

    t = time.perf_counter()
    mask = fixed_size_disk(RADIUS)
    time_report[f"Time to instantiate a disk of radius {RADIUS}"] = time.perf_counter() - t

    t = time.perf_counter()
    obs = cv2.dilate(
        obs,
        np.uint8(mask),
        borderValue=1,
        iterations=1,
    )
    time_report[f"Dilation time"] = time.perf_counter() - t

    width, height = obs.shape

    t = time.perf_counter()
    grid = obs.astype(bool, copy=False)
    time_report[f"Casting time"] = time.perf_counter() - t

    t = time.perf_counter()
    exit_path = grid_pathfinding.exit_red_zone(grid, START)
    time_report[f"Exit red zone time"] = time.perf_counter() - t

    t = time.perf_counter()
    path_to_point = grid_pathfinding.find_path(grid, exit_path, END)
    time_report[f"Find path"] = time.perf_counter() - t

    max_length = max(map(len, time_report.keys()))
    time_report['SUM'] = sum(time_report.values())
    print(f"{'OPERATION NAME':>{max_length}}  TIME [ms]")
    for name, value in time_report.items():
        print(f"{name:>{max_length}}: {value * 1000:06.3f}")

    print(f"Exit path: {exit_path}")
    print(f"Path to point: {path_to_point}")

    real_path = []
    for i in range(1, len(path_to_point)):
        for pos in bresenham(*path_to_point[i - 1], *path_to_point[i]):
            real_path.append(pos)

    if 'PRINT_GRID' in os.environ:
        output = ''
        for y in range(height):
            for x in range(width):
                character = ' '
                if (x, y) == START:
                    character = 'S'
                elif (x, y) == END:
                    character = 'E'
                elif (x, y) == exit_path:
                    character = 'X'
                elif (x, y) in path_to_point:
                    character = '#'
                elif (x, y) in real_path:
                    character = 'O'
                elif grid[x, y]:
                    character = '#'
                output += character + ' '

            output += '\n'
        print(output)
