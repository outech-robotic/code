"""
Trajectory controller module.
"""
from highlevel.util.geometry.vector import Vector2


class TrajectoryController:
    """
    Trajectory controller, allows movements between two points, using a pathfinding algorithm
    """
    async def move_to(self, target: Vector2, reverse: bool) -> None:
        """
        Moves the robot to a new position, avoiding obstacles in the way
        """
