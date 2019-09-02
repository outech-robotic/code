"""
Test for simulation gateway module.
"""


def test_movement_done(simulation_gateway, motion_handler):
    """
    Should call movement_done on motion handler.
    """
    simulation_gateway.movement_done()
    motion_handler.movement_done.assert_called_once()


def test_update_location_update_motors(simulation_gateway, motion_handler,
                                       simulation_state_repository):
    """
    Should call position_update on motion handler.
    """
    pos = simulation_state_repository.robot_position
    angle = simulation_state_repository.robot_angle
    simulation_gateway.update_location()
    motion_handler.position_update.assert_called_once_with(pos.x, pos.y, angle)


def test_update_location_sensors(simulation_gateway, distance_sensor_handler):
    """
    Should call the sensor handlers with the right distance values.
    """
    simulation_gateway.update_location()

    distance_sensor_handler.distance_forward.assert_called_once_with(75)
    distance_sensor_handler.distance_backward.assert_called_once_with(25)
    distance_sensor_handler.distance_left.assert_called_once_with(75)
    distance_sensor_handler.distance_right.assert_called_once_with(25)
