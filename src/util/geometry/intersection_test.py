"""
Test for geometry module.
"""
from src.robot.entity.motion.geometry import Segment, Ray
from src.robot.entity.motion.vector import Vector2
from src.util.geometry.intersection import (ray_segment_intersection,
                                            ray_segments_intersection,
                                            segment_segment_intersection,
                                            does_segment_intersect)


def test_ray_segment_intersection_colinear():
    """
    Ray and segment are colinear, should not intersect.

    Ascii:
    +-------> Ray
    +-------------------------+ Segment
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

              Ray
               +
               |
               |
               v
    +----------+-----------+ Segment

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


def test_ray_segment_intersection_ray_not_in_the_right_direction():
    """
    Ray does not aim at the right direction.

              Ray
               ^
               |
               |

    +----------------------+ Segment

    """
    seg = Segment(
        start=Vector2(0, 0),
        end=Vector2(2, 0),
    )
    ray = Ray(
        origin=Vector2(1, 1),
        direction=Vector2(0, 1),
    )

    pos, _ = ray_segment_intersection(ray, seg)
    assert pos is None


def test_ray_segment_intersection_out_of_bounds_left():
    """
    Ray is too far on the left, do not intersect.

    Ray
     +
     |
     |
     |
     v     +----------------------+ Segment

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

                                      Ray
                                       +
                                       |
                                       |
                                       |
    Segment +----------------------+   v

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


def test_ray_segment_intersection_0_length_segment():
    """
    Ray is too far on the right, do not intersect (by convention).

           Ray
            +
            |
            |
            v
    Segment +

    """
    seg = Segment(
        start=Vector2(0, 0),
        end=Vector2(0, 0),
    )
    ray = Ray(
        origin=Vector2(0, 1),
        direction=Vector2(0, -1),
    )

    pos, _ = ray_segment_intersection(ray, seg)
    assert pos is None


def test_ray_segments_intersection_intersect_closest():
    """
    Happy path.

    +----------------+ Segment 0
    +----------------+ Segment 1
            ^
            | Ray
            +
    """

    pos, dist = ray_segments_intersection(
        Ray(origin=Vector2(0, 0), direction=Vector2(0, 1)), [
            Segment(start=Vector2(-1, 2), end=Vector2(1, 2)),
            Segment(start=Vector2(-1, 1), end=Vector2(1, 1)),
        ])

    assert pos == Vector2(0, 1)
    assert dist == 1


def test_ray_segments_intersection_does_not_intersect():
    """
    Happy path.

            ^
            | Ray
            +
    +----------------+ Segment 0
    """

    pos, dist = ray_segments_intersection(
        Ray(origin=Vector2(0, 0), direction=Vector2(0, 1)), [
            Segment(start=Vector2(-1, -2), end=Vector2(1, -2)),
        ])

    assert pos is None
    assert dist is None


def test_segment_segment_intersection_intersect():
    """
    Happy path.

         +
         |
         |
    +----+----+
         |
         |
         +
    """

    pos = segment_segment_intersection(
        Segment(Vector2(-1, 0), Vector2(1, 0)),
        Segment(Vector2(0, -1), Vector2(0, 1)),
    )

    assert pos == Vector2(0, 0)


def test_segment_segment_intersection_not_intersect():
    """
    Segments do not intersect.

         +
         |
         +
         
    +---------+
    """

    pos = segment_segment_intersection(
        Segment(Vector2(-1, 0), Vector2(1, 0)),
        Segment(Vector2(0, 2), Vector2(0, 1)),
    )

    assert pos is None


def test_segment_segment_intersection_not_intersect_colinear():
    """
    Segments do not intersect.
    
    +---------+
    
    +---------+
    """

    pos = segment_segment_intersection(
        Segment(Vector2(-1, 0), Vector2(1, 0)),
        Segment(Vector2(-1, 1), Vector2(1, 1)),
    )

    assert pos is None


def test_does_segment_intersect_happy_path():
    """
    Happy path
    
         +
         |
         |
    +----+----+ segment 1
         |
         |
         +
    +---------+
    
    +---------+
    """

    intersect = does_segment_intersect(
        Segment(Vector2(-1, 0), Vector2(1, 0)),
        [
            Segment(Vector2(-1, -2), Vector2(1, -2)),
            Segment(Vector2(0, 1), Vector2(0, -1)),
        ],
    )

    assert intersect is True


def test_does_segment_intersect():
    """
    Happy path
    
    +---------+ segment 1
    +---------+
    +---------+
    """

    intersect = does_segment_intersect(
        Segment(Vector2(-1, 0), Vector2(1, 0)),
        [
            Segment(Vector2(-1, 1), Vector2(1, 1)),
            Segment(Vector2(-1, -2), Vector2(1, -2)),
        ],
    )

    assert intersect is False
