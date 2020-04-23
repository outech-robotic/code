"""
Test probe.
"""
from unittest.mock import MagicMock

import pytest

from highlevel.util.probe import Probe, DebugEvent


@pytest.fixture(name='probe')
def setup_probe(clock_mock):
    """
    Setup test subject.
    """
    probe = Probe(clock=clock_mock)

    clock_mock.time = MagicMock(return_value=0)
    probe.emit('my_value', 1)

    clock_mock.time = MagicMock(return_value=10)
    probe.emit('my_metric', 2)

    clock_mock.time = MagicMock(return_value=20)
    probe.emit('my_metric', 3)
    return probe


class TestProbe:
    @staticmethod
    def test_poll(probe, clock_mock):
        """
        Make sure polling works.
        :return:
        """
        got, cursor = probe.poll()
        assert got == [
            DebugEvent(
                time=0,
                key='my_value',
                value=1,
            ),
            DebugEvent(
                time=10,
                key='my_metric',
                value=2,
            ),
            DebugEvent(
                time=20,
                key='my_metric',
                value=3,
            ),
        ]

        clock_mock.time = MagicMock(return_value=30)
        probe.emit('some_value', 42)

        got, cursor = probe.poll(cursor=cursor)
        assert got == [
            DebugEvent(
                time=30,
                key='some_value',
                value=42,
            ),
        ]

    @staticmethod
    def test_poll_with_downrate(probe):
        """
        Make sure downrating event works.
        """
        got, _ = probe.poll(rate=1/20)
        assert got == [
            DebugEvent(
                time=0,
                key='my_value',
                value=1,
            ),
            DebugEvent(
                time=10,
                key='my_metric',
                value=2,
            ),
        ]
