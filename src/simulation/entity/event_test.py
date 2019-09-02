"""
Test for event module.
"""
from src.simulation.entity.event import EventQueue, Event, EventOrder, EventType


def test_queue_happy_path():
    """
    Happy path.
    """
    event_queue = EventQueue()
    event1 = Event(tick=0,
                   event=EventOrder(
                       type=EventType.MOVE_FORWARD,
                       payload=42,
                   ))
    event2 = Event(tick=0,
                   event=EventOrder(
                       type=EventType.MOVE_FORWARD,
                       payload=24,
                   ))
    event3 = Event(tick=2, event=EventOrder(
        type=EventType.ROTATE,
        payload=42,
    ))

    event_queue.push(event3)
    event_queue.push(event2)
    event_queue.push(event1)

    assert event_queue.pop(0) == [event1, event2]
    assert event_queue.pop(1) == []
    assert event_queue.pop(2) == [event3]


def test_queue_pop_remove_elements():
    """
    Pop should remove elements from the queue.
    """
    event_queue = EventQueue()
    event1 = Event(tick=0,
                   event=EventOrder(
                       type=EventType.MOVE_FORWARD,
                       payload=42,
                   ))

    event_queue.push(event1)

    assert event_queue.pop(0)
    assert not event_queue.pop(0)
