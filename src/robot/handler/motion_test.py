"""
Test motion handler module.
"""


def test_position_update(motion_handler, localization_controller):
    """
    Happy path.
    """
    motion_handler.position_update(1, 2, 3)
    localization_controller.update_odometry_position.asser_called_once_with(
        1, 2, 3)


def test_movement_done(motion_handler, localization_controller):
    """
    Happy path.
    """
    motion_handler.movement_done()
    localization_controller.set_is_moving.assert_called_with(False)
