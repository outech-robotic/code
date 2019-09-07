"""
Test for simulation controller module.
"""
from src.simulation.entity.event import EventType


def test_robot_move_forward(simulation_controller, event_queue,
                            simulation_configuration):
    """
    Happy path, will create N move forward event for each tick.
    """
    distance = simulation_configuration.translation_speed
    ticks = simulation_configuration.tickrate

    simulation_controller.robot_move_forward(distance)

    for i in range(ticks):
        event_list = event_queue.pop(i)
        assert len(event_list) == 1

        event = event_list[0].event
        assert event.type == EventType.MOVE_FORWARD
        assert event.payload == distance / ticks


def test_robot_move_forward_send_movement_done(simulation_controller,
                                               event_queue,
                                               simulation_configuration):
    """
    Ensure the "movement done" event is sent after translation is done.
    """
    distance = simulation_configuration.translation_speed
    ticks = simulation_configuration.tickrate

    simulation_controller.robot_move_forward(distance)

    # Discard all the other events from previous ticks.
    event_queue.pop(ticks - 1)

    final_event_list = event_queue.pop(ticks)
    assert len(final_event_list) == 1
    assert final_event_list[0].event.type == EventType.MOVEMENT_DONE


def test_robot_rotate(simulation_controller, event_queue,
                      simulation_configuration):
    """
    Happy path, will create N rotate events for each tick.
    """
    angle = simulation_configuration.rotation_speed
    ticks = simulation_configuration.tickrate

    simulation_controller.robot_rotate(angle)

    for i in range(ticks):
        event_list = event_queue.pop(i)
        assert len(event_list) == 1

        event = event_list[0].event
        assert event.type == EventType.ROTATE
        assert event.payload == angle / ticks


def test_robot_rotate_send_movement_done(simulation_controller, event_queue,
                                         simulation_configuration):
    """
    Ensure the "movement done event" is sent after rotation is done.
    """
    angle = simulation_configuration.rotation_speed
    ticks = simulation_configuration.tickrate

    simulation_controller.robot_rotate(angle)
    # Discard all the other events from previous ticks.

    event_queue.pop(ticks - 1)

    final_event_list = event_queue.pop(ticks)
    assert len(final_event_list) == 1
    assert final_event_list[0].event.type == EventType.MOVEMENT_DONE
