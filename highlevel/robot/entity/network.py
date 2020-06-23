"""
Network configuration module.
Includes every bus slave and its sub data.
"""
from enum import Enum

from highlevel.adapter.socket.isotp import ISOTPAddress


def net_get_tx_id(start_id: int, index: int) -> int:
    return 2 * (start_id + index)


def net_get_rx_id(tx_id: int) -> int:
    return tx_id + 1


CAN_DEVICE = "can0"
NB_SERVO_BOARDS = 7

# ISOTP ADDRESS OBJECTS FOR BOARDS
NET_MOTOR_BOARD = ISOTPAddress(CAN_DEVICE, 1, 0)
NET_SERVO_BOARDS = [ISOTPAddress(CAN_DEVICE,
                                 net_get_rx_id(net_get_tx_id(0x10, i)),
                                 net_get_tx_id(0x10, i)
                                 ) for i in range(0, NB_SERVO_BOARDS)]

class BOARD_IDS(Enum):
    """
    Board transmission IDs, used as they ID in this program.
    To get the reception id, add 1.
    """
    MOTOR = 0x00
    ARM_RIGHT = 0x10 * 2
    ARM_CENTER_RIGHT = 0x11 * 2
    ARM_CENTER = 0x12 * 2
    ARM_CENTER_LEFT = 0x13 * 2
    ARM_LEFT = 0x14 * 2
