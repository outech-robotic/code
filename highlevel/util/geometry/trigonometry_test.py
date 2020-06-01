"""
Trigonometry module tests.
"""
import math

import pytest

from highlevel.util.geometry.trigonometry import normalize_angle



@pytest.mark.parametrize(
    'angle,expected',
    [   # these should not be changed
        (0,0),
        (3, 3),
        (-3, -3),
        (math.pi, math.pi),
        # These should be mapped back to ]-pi;pi]
        (-math.pi, math.pi),
        (math.pi+5, math.pi + 5 - 2*math.pi)
    ])
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == expected
