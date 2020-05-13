"""
Test for symmetry controller
"""
import dataclasses
from math import pi

from highlevel.robot.controller.symmetry import SymmetryController
from highlevel.robot.entity.color import Color
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.geometry.vector import Vector2


def get_symmetry_controller(
        color: Color, configuration_test: Configuration) -> SymmetryController:
    """
    Return a symmetry controller configured with the given color.
    """
    return SymmetryController(
        configuration=dataclasses.replace(configuration_test, color=color))


def test_vector_sym1(configuration_test: Configuration) -> None:
    """
    Test vector symmetry
    """
    vec = Vector2(-148, 29)
    symmetry_controller = get_symmetry_controller(Color.YELLOW,
                                                  configuration_test)
    sym_vec = symmetry_controller.symmetries_position(vec)

    assert sym_vec.x == 148
    assert sym_vec.y == 29


def test_vector_sym2(configuration_test: Configuration) -> None:
    """        
    Test vector symmetry
    """
    vec = Vector2(-167, 124)
    symmetry_controller = get_symmetry_controller(Color.BLUE,
                                                  configuration_test)
    sym_vec = symmetry_controller.symmetries_position(vec)

    assert sym_vec.x == -167
    assert sym_vec.y == 124


def test_angle_sym1(configuration_test: Configuration) -> None:
    """
    Test angle symmetry
    """
    angle = pi / 8
    symmetry_controller = get_symmetry_controller(Color.YELLOW,
                                                  configuration_test)
    sym_angle = symmetry_controller.symmetries_rotate(angle)

    assert sym_angle == 7 * pi / 8


def test_angle_sym2(configuration_test: Configuration) -> None:
    """
    Test angle symmetry
    """
    angle = pi / 8
    symmetry_controller = get_symmetry_controller(Color.BLUE,
                                                  configuration_test)
    sym_angle = symmetry_controller.symmetries_rotate(angle)

    assert sym_angle == pi / 8
