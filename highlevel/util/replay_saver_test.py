"""
Test for replay saver module.
"""
import json
from unittest.mock import MagicMock

from highlevel.adapter.http import HTTPClient
from highlevel.adapter.web_browser import WebBrowserClient
from highlevel.util.replay_saver import ReplaySaver
from highlevel.util.probe import DebugEvent


def test_happy_path(simulation_configuration_test, configuration_test,
                    probe_mock):
    """
    Happy path.
    """

    http_client = MagicMock(spec=HTTPClient)
    http_client.post_file = MagicMock(return_value={'id': 'test_id'})

    replay_saver = ReplaySaver(
        configuration=configuration_test,
        simulation_configuration=simulation_configuration_test,
        http_client=http_client,
        web_browser_client=MagicMock(spec=WebBrowserClient),
        probe=probe_mock,
    )

    probe_mock.poll = MagicMock(return_value=([
        DebugEvent(
            key="position",
            time=0,
            value={
                "x": 0,
                "y": 0,
            },
        ),
        DebugEvent(
            key="angle",
            time=0,
            value=42,
        ),
    ], 42))
    replay_saver.save_replay()

    http_client.post_file.assert_called_once()

    # Make sure the 2nd argument of the call is a valid JSON string and that it conforms to the
    # JSON expected output.
    data = http_client.post_file.call_args_list[0][0][1]
    assert json.loads(data)['events'][2:] == [{
        'time': 0,
        'key': 'position',
        'value': {
            'x': 0,
            'y': 0
        },
    }, {
        'time': 0,
        'key': 'angle',
        'value': 42,
    }]
