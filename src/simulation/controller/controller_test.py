"""
Test for simulation controller module.
"""
from pytest import fixture

from src.simulation.controller.controller import SimulationController
from src.simulation.entity.event import EventType
from src.simulation.entity.state import RobotID


@fixture(name='simulation_controller')
def simulation_controller_factory(event_queue_mock,
                                  simulation_configuration_test):
    """
    Simulation controller.
    """
    return SimulationController(
        event_queue=event_queue_mock,
        simulation_configuration=simulation_configuration_test,
    )


def test_robot_move_forward(simulation_controller, event_queue_mock,
                            simulation_configuration_test):
    """
    Happy path, will create N move forward event for each tick.
    """
    distance = simulation_configuration_test.translation_speed
    ticks = simulation_configuration_test.tickrate

    simulation_controller.robot_move_forward(distance, RobotID.RobotA)

    for _ in range(ticks):
        event_list = list(event_queue_mock.pop())
        assert len(event_list) == 1

        event = event_list[0]
        assert event.type == EventType.MOVE_FORWARD
        assert event.payload.get('distance') == distance / ticks
        assert event.payload.get('robot_id') == RobotID.RobotA


def test_robot_move_forward_send_movement_done(simulation_controller,
                                               event_queue_mock,
                                               simulation_configuration_test):
    """
    Ensure the "movement done" event is sent after translation is done.
    """
    distance = simulation_configuration_test.translation_speed
    ticks = simulation_configuration_test.tickrate

    simulation_controller.robot_move_forward(distance, RobotID.RobotA)

    # Discard all the other events from previous ticks.
    for _ in range(ticks):
        event_queue_mock.pop()

    final_event_list = list(event_queue_mock.pop())
    assert len(final_event_list) == 1
    assert final_event_list[0].type == EventType.MOVEMENT_DONE


def test_robot_rotate(simulation_controller, event_queue_mock,
                      simulation_configuration_test):
    """
    Happy path, will create N rotate events for each tick.
    """
    angle = simulation_configuration_test.rotation_speed
    ticks = simulation_configuration_test.tickrate

    simulation_controller.robot_rotate(angle, RobotID.RobotA)

    for _ in range(ticks):
        event_list = list(event_queue_mock.pop())
        assert len(event_list) == 1

        event = event_list[0]
        assert event.type == EventType.ROTATE
        assert event.payload.get('angle') == angle / ticks
        assert event.payload.get('robot_id') == RobotID.RobotA


def test_robot_rotate_send_movement_done(simulation_controller,
                                         event_queue_mock,
                                         simulation_configuration_test):
    """
    Ensure the "movement done event" is sent after rotation is done.
    """
    angle = simulation_configuration_test.rotation_speed
    ticks = simulation_configuration_test.tickrate

    simulation_controller.robot_rotate(angle, RobotID.RobotA)

    # Discard all the other events from previous ticks.
    for _ in range(ticks):
        event_queue_mock.pop()

    final_event_list = event_queue_mock.pop()
    assert len(final_event_list) == 1
    assert final_event_list[0].type == EventType.MOVEMENT_DONE
