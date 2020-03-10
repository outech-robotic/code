"""
JSON encoder module.
"""
import json

from src.robot.entity.motion.vector import Vector2


class RobotJSONEncoder(json.JSONEncoder):
    """
    Custom JSON encoder, able to encode our own entities.
    """

    # pylint: disable=method-hidden
    def default(self, o):
        if isinstance(o, Vector2):
            return [float(o.x), float(o.y)]
        return json.JSONEncoder.default(self, o)
