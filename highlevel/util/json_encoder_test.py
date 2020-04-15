"""
Test for JSON encoder.
"""
import json

from pytest import raises

from highlevel.util.geometry.vector import Vector2
from highlevel.util.json_encoder import RobotJSONEncoder


def test_json_encoder_vector2():
    """
    Happy path for Vector2.
    """

    to_encode = {'a': Vector2(2, 3)}

    dump = json.dumps(to_encode, cls=RobotJSONEncoder, indent=None)
    assert dump == '{"a": {"x": 2.0, "y": 3.0}}'


def test_json_encoder_not_serializable():
    """
    Make sure the encoder will try to encode the other types.
    """
    class NotSerializable:
        """
        A class that is not JSON serializable.
        """

    to_encode = {'a': NotSerializable()}

    with raises(TypeError):
        json.dumps(to_encode, cls=RobotJSONEncoder)
