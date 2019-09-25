"""
Test for JSON encoder.
"""
import json

from src.robot.entity.vector import Vector2
from src.util.json_encoder import RobotJSONEncoder


def test_json_encoder_vector2():
    """
    Happy path for Vector2.
    """

    to_encode = {'a': Vector2(2, 3)}

    dump = json.dumps(to_encode, cls=RobotJSONEncoder, indent=None)
    assert dump == '{"a": [2.0, 3.0]}'


def test_json_encoder_normal_types():
    """
    Make sure the encoder can still encode "normal" types.
    """

    to_encode = {'a': 'hello', 'b': 1}

    dump = json.dumps(to_encode,
                      cls=RobotJSONEncoder,
                      indent=None,
                      sort_keys=True)
    assert dump == '{"a": "hello", "b": 1}'
