"""
Test for geometry module.
"""
from src.entity.geometry import Segment, Ray
from src.entity.vector import Vector2
from src.util.geometry import ray_segment_intersection


def test_ray_segment_intersection_colinear():
    """
    Ray and segment are colinear, should not intersect.
    """
    seg = Segment(
        start=Vector2(0, 0),
        end=Vector2(100, 0),
    )
    ray = Ray(
        origin=Vector2(0, 1),
        direction=Vector2(1, 0),
    )

    pos, _ = ray_segment_intersection(ray, seg)
    assert pos is None


def test_ray_segment_intersection_happy_path():
    """
    Happy path, intersect.
    """
    seg = Segment(
        start=Vector2(0, 0),
        end=Vector2(2, 0),
    )
    ray = Ray(
        origin=Vector2(1, 1),
        direction=Vector2(0, -1),
    )

    pos, _ = ray_segment_intersection(ray, seg)
    assert pos == Vector2(1, 0)


def test_ray_segment_intersection_out_of_bounds_left():
    """
    Ray is too far on the left, do not intersect.
    """
    seg = Segment(
        start=Vector2(0, 0),
        end=Vector2(2, 0),
    )
    ray = Ray(
        origin=Vector2(10, 1),
        direction=Vector2(0, -1),
    )

    pos, _ = ray_segment_intersection(ray, seg)
    assert pos is None


def test_ray_segment_intersection_out_of_bounds_right():
    """
    Ray is too far on the right, do not intersect.
    """
    seg = Segment(
        start=Vector2(0, 0),
        end=Vector2(2, 0),
    )
    ray = Ray(
        origin=Vector2(-10, 1),
        direction=Vector2(0, -1),
    )

    pos, _ = ray_segment_intersection(ray, seg)
    assert pos is None
