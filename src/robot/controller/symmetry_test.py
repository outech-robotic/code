"""
Test for symmetry controller
"""
from math import pi

from src.robot.controller.symmetry import SymmetryController
from src.robot.entity.color import Color
from src.robot.entity.configuration import Configuration
from src.robot.entity.vector import Vector2


def get_symmetry_controller(color: Color) -> SymmetryController:
    """
    Return a symmetry controller configured with the given color.
    """
    return SymmetryController(
        Configuration(
            initial_position=Vector2(1500, 1000),
            initial_angle=0,
            robot_width=330,
            robot_length=330,
            field_shape=(3000, 2000),
            color=color,
        ))


def test_vector_sym1():
    """
    Test vector symmetry
    """
    vec = Vector2(-148, 29)
    symmetry_controller = get_symmetry_controller(color=Color.YELLOW)
    sym_vec = symmetry_controller.symmetries_position(vec)

    assert sym_vec.x == 148
    assert sym_vec.y == 29


def test_vector_sym2():
    """        
    Test vector symmetry
    """
    vec = Vector2(-167, 124)
    symmetry_controller = get_symmetry_controller(color=Color.BLUE)
    sym_vec = symmetry_controller.symmetries_position(vec)

    assert sym_vec.x == -167
    assert sym_vec.y == 124


def test_angle_sym1():
    """
    Test angle symmetry
    """
    angle = pi / 8
    symmetry_controller = get_symmetry_controller(color=Color.YELLOW)
    sym_angle = symmetry_controller.symmetries_rotate(angle)

    assert sym_angle == 7 * pi / 8


def test_angle_sym2():
    """
    Test angle symmetry
    """
    angle = pi / 8
    symmetry_controller = get_symmetry_controller(color=Color.BLUE)
    sym_angle = symmetry_controller.symmetries_rotate(angle)

    assert sym_angle == pi / 8
