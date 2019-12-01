"""
Intersection module.
"""
import math
from typing import Tuple, Optional, Iterable

import numpy

from src.robot.entity.geometry import Ray, Segment
from src.robot.entity.vector import Vector2


def ray_segment_intersection(
        ray: Ray,
        segment: Segment) -> Tuple[Optional[Vector2], Optional[float]]:
    """
    Get the intersection point between a ray and a segment. 
    More precisely, will find the intersection point between [a,b) and  ]c,d[ (thus a 0 length 
    segment will never intersect).

    Returns None if they do not intersect. 
    Return the intersection coordinates and the distance from the origin of the ray otherwise.
    """

    origin = numpy.array([ray.origin.x, ray.origin.y])
    direction = numpy.array([ray.direction.x, ray.direction.y])

    start = numpy.array([segment.start.x, segment.start.y])
    end = numpy.array([segment.end.x, segment.end.y])

    # Find intersection point:
    # origin + direction * x = start + (end - start) * y
    # origin ^ (end-start) + direction ^ (end-start) * x = start ^ (end-start)
    # direction ^ (end-start) * x = start ^ (end-start) - origin ^ (end-start)
    # direction ^ (end-start) * x = (start - origin) ^ (end-start)
    # A * x = B
    # x = B / A
    segment_length2 = (end - start).dot(end - start)
    if segment_length2 == 0:
        return None, None  # Segment is 0 length, will never be hit by a ray.

    coef_a = numpy.cross(direction, (end - start))
    if coef_a == 0:
        return None, None  # Collinear, will never intersect.

    coef_b = numpy.cross(start - origin, end - start)
    v_x = coef_b / coef_a
    if v_x < 0:
        return None, None  # Ray hit behind.

    # Then...
    # intersection_point = origin + direction * x
    intersection_point = origin + direction * v_x

    # To find Y:
    # intersection_point = start + (end - start) * y
    # intersection_point - start = (end - start) * y
    # (intersection_point - start) . (end - start) = (end - start) . (end - start) * y
    # (intersection_point - start) . (end - start) = || end - start || ** 2 * y
    # y = (intersection_point - start) . (end - start) / || end - start || ** 2
    # y = (intersection_point - start) . (end - start) / || end - start || ** 2
    coef_a = (intersection_point - start).dot(end - start)
    v_y = coef_a / segment_length2
    if v_y <= 0 or v_y >= 1:
        return None, None  # Ray did not hit segment.

    return Vector2(*intersection_point), v_x


def ray_segments_intersection(
    ray: Ray, segments: Iterable[Segment]
) -> Tuple[Optional[Vector2], Optional[float]]:
    """
    Compute the intersection of a ray and a set of segments.
    """
    closest_distance = math.inf
    closest_point = None

    for seg in segments:
        point, distance = ray_segment_intersection(ray, seg)
        if distance is None:
            continue
        if distance < closest_distance:
            closest_distance = distance
            closest_point = point

    if closest_point is None:
        return None, None

    return closest_point, closest_distance


def segment_segment_intersection(sgmt1: Segment,
                                 sgmt2: Segment) -> Optional[Vector2]:
    """
    Check if a segment intersect with another segment.
    """
    origin = sgmt2.start

    direction = sgmt2.end - sgmt2.start
    distance = direction.euclidean_norm()

    direction_normalized = direction / distance
    ray = Ray(origin=origin, direction=direction_normalized)

    pos, ray_dist = ray_segment_intersection(ray, sgmt1)
    if ray_dist is None:
        return None

    if ray_dist > distance:
        return None  # Too far away on the ray.

    return pos


def does_segment_intersect(sgmt1: Segment, segments: Iterable[Segment]) -> bool:
    """
    Check if a segment intersects with a set of other segments.
    """
    for sgmt2 in segments:
        pos = segment_segment_intersection(sgmt1, sgmt2)
        if pos is not None:
            return True

    return False
