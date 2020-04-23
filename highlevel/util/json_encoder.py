"""
JSON encoder module.
"""
import json
from dataclasses import asdict

from highlevel.robot.entity.color import Color
from highlevel.util.geometry.vector import Vector2
from highlevel.util.probe import DebugEvent


class RobotJSONEncoder(json.JSONEncoder):
    """
    Custom JSON encoder, able to encode our own entities.
    """

    # pylint: disable=method-hidden
    def default(self, o):
        if isinstance(o, Vector2):
            return {"x": float(o.x), "y": float(o.y)}
        if isinstance(o, Color):
            return o.name
        if isinstance(o, DebugEvent):
            return asdict(o)
        return json.JSONEncoder.default(self, o)
