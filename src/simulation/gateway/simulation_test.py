"""
Test for simulation gateway module.
"""


def test_movement_done(simulation_gateway, motion_handler):
    """
    Should call movement_done on motion handler.
    """
    simulation_gateway.movement_done()
    motion_handler.movement_done.assert_called_once()
