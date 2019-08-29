"""
Animation module.
"""
# Pylint does not find the pygame methods, disabling the inspection for this module.
# pylint: disable=no-member,too-many-function-args
import math
from typing import Tuple

import pygame as py

from src.controller.localization import LocalizationController
from src.controller.map import MapController
from src.entity.geometry import Ray
from src.entity.vector import Vector2
from src.simulation.simulation import Simulation
from src.util.geometry import ray_segments_intersection, forward, backward, left, right

MARGIN = 15

WIDTH = 1000
HEIGHT = WIDTH * 200 // 300

FPS = 60

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
GREY = (60, 60, 60)
DARK_GREEN = (0, 150, 0)
DARK_RED = (150, 0, 0)

BACKGROUND_COLOR = GREY


def x(x_pos: float) -> int:  # Allow single character name. pylint: disable=invalid-name
    """
    Map from Millimeters to pixels.
    """
    return int(x_pos * WIDTH // 300)


def y(y_pos: float) -> int:  # Allow single character name. pylint: disable=invalid-name
    """
    Map from Millimeters to pixels.
    """
    return int(y_pos * HEIGHT // 200)


def pos(vec: Vector2) -> Tuple:
    """
    Map position from Millimeters to pixels (also add a margin).
    """
    return x(vec.x) + MARGIN, y(vec.y) + MARGIN


def size(vec: Vector2) -> Tuple:
    """
    Map size from Millimeters to pixels.
    """
    return x(vec.x), y(vec.y)


ROBOT_SIZE = size(Vector2(30, 30))


def _should_quit() -> bool:
    for event in py.event.get():
        if event.type == py.QUIT:
            return True

        if event.type == py.KEYDOWN:
            if event.key == py.K_q:
                return True

            if event.key == py.K_c and py.key.get_mods() & py.KMOD_CTRL:
                return True
    return False


class Animation:
    """
    An animation is a representation of what is happening in a simulation versus what is happening
    according to the robot processing.
    """
    def __init__(self, simulation: Simulation,
                 localization_controller: LocalizationController,
                 map_controller: MapController):
        self.simulation = simulation
        self.localization_controller = localization_controller
        self.map_controller = map_controller

        # ROBOT IMG: (real position of the robot)
        self.robot_surface = py.Surface(ROBOT_SIZE)
        self.robot_surface.set_colorkey(BLACK)
        self.robot_surface.fill(DARK_GREEN)

        # SHADOW ROBOT IMG: (internal position calculated by the robot)
        self.calculated_robot_surface = py.Surface(ROBOT_SIZE)
        self.calculated_robot_surface.set_colorkey(BLACK)
        self.calculated_robot_surface.fill(RED)
        self.calculated_robot_surface.set_alpha(150)

        # GRID CELL IMG: (Internal obstacle in the obstacle grid of the robot)
        cell_size = size(Vector2(10, 10))
        self.cell_size = cell_size
        self.grid_cell = py.Surface(cell_size)
        self.grid_cell.set_alpha(128 / 2)
        self.grid_cell.fill((255, 99, 71))
        py.draw.rect(self.grid_cell, BLACK, (0, 0, *cell_size), 2)  # Borders.
        py.draw.line(self.grid_cell, BLACK, (0, 0), cell_size, 3)  # Cross /
        py.draw.line(self.grid_cell, BLACK, (cell_size[0], 0),
                     (0, cell_size[1]), 3)  # Cross \

    def _draw(self, screen):
        _draw_field(screen)
        self._draw_laser_sensors(screen)
        self._draw_robot(screen)
        self._draw_obstacles(screen)
        self._draw_obstacle_grid(screen)

    def _draw_laser_sensors(self, screen):
        directions = [
            (forward(self.simulation.angle), BLACK),
            (backward(self.simulation.angle), GREEN),
            (right(self.simulation.angle), RED),
            (left(self.simulation.angle), BLUE),
        ]
        for direction, color in directions:
            start = self.simulation.position
            ray = Ray(start, direction)
            end, _ = ray_segments_intersection(ray, self.simulation.obstacles)
            if end:
                py.draw.line(screen, color, pos(start), pos(end), 1)

    def _draw_robot(self, screen):
        new_image = py.transform.rotate(self.robot_surface,
                                        -self.simulation.angle * 180 / math.pi)
        rect = new_image.get_rect()
        robot_pos = self.simulation.position
        rect.center = pos(robot_pos)
        screen.blit(new_image, rect)

        # Draw the shadow of the robot, the position as seen by the robot.
        # dir = self.localization_controller.get_direction()
        # angle = math.acos(dir.dot(Vector2(1, 0)))
        angle = self.localization_controller.localization_repository.odometry_direction
        new_image = py.transform.rotate(self.calculated_robot_surface,
                                        -angle * 180 / math.pi)
        rect = new_image.get_rect()
        robot_pos = self.localization_controller.get_position()
        rect.center = pos(robot_pos)
        screen.blit(new_image, rect)

    def _draw_obstacles(self, screen):
        for obstacle in self.simulation.obstacles:
            py.draw.line(screen, WHITE, pos(obstacle.start), pos(obstacle.end),
                         1)

    def _draw_obstacle_grid(self, screen):
        obstacle_grid = self.map_controller.map_repository.map
        for i in range(obstacle_grid.shape[0]):
            for j in range(obstacle_grid.shape[1]):
                if obstacle_grid[i, j]:
                    position = (x(i * 10), y(j * 10))
                    screen.blit(self.grid_cell, position)

    def render(self):
        """
        Render the animation.
        """

        py.init()
        screen = py.display.set_mode((WIDTH + MARGIN * 2, HEIGHT + MARGIN * 2))
        clock = py.time.Clock()

        running = True
        while running:
            clock.tick(FPS)

            self._draw(screen)

            py.display.flip()
            if _should_quit():
                running = False

        py.quit()


def _draw_field(screen):
    screen.fill(BLACK)
    py.draw.rect(screen, GREY, (MARGIN, MARGIN, WIDTH, HEIGHT))
