"""
Test for event queue module.
"""
from highlevel.simulation.controller.event_queue import EventQueue
from highlevel.simulation.entity.event import EventOrder, EventType


def test_queue_happy_path():
    """
    Happy path.
    """
    event_queue = EventQueue()
    event1 = EventOrder(
        type=EventType.MOVE_WHEEL,
        payload=42,
    )
    event2 = EventOrder(
        type=EventType.MOVE_WHEEL,
        payload=24,
    )
    event3 = EventOrder(
        type=EventType.MOVEMENT_DONE,
        payload=42,
    )

    event_queue.push(event1, 2)
    event_queue.push(event2, 0)
    event_queue.push(event3, 0)

    assert set(event_queue.pop()) == {event2, event3}
    assert set(event_queue.pop()) == set()
    assert set(event_queue.pop()) == {event1}
