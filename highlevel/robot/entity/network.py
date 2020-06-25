"""
Network configuration module.
Includes every bus slave and its sub data.
"""
from enum import Enum

from highlevel.adapter.socket.isotp import ISOTPAddress


def _net_get_tx_id(id_offset: int, index: int) -> int:
    """
    Returns the transmission id of a board, given its type's start_id and its index.
    """
    return 2 * (id_offset + index)


def _net_get_rx_id(tx_id: int) -> int:
    """
    Returns the reception id of a board given its transmission id.
    """
    return tx_id + 1


_CAN_DEVICE = "can0"
NB_SERVO_BOARDS = 7

_ID_OFFSET_MOTOR = 0x00
_ID_OFFSET_SERVO = 0x10

# ISOTP ADDRESS OBJECTS FOR BOARDS
NET_ADDRESS_MOTOR = ISOTPAddress(_CAN_DEVICE, 1, 0)
NET_ADDRESSES_SERVO = [
    ISOTPAddress(_CAN_DEVICE, _net_get_rx_id(_net_get_tx_id(0x10, i)),
                 _net_get_tx_id(0x10, i)) for i in range(0, NB_SERVO_BOARDS)
]


class BoardID(Enum):
    """
    Board transmission IDs, used as they ID in this program.
    To get the reception id, use net_get_rx_id.
    """
    MOTOR = _net_get_tx_id(_ID_OFFSET_MOTOR, 0)

    ARM_RIGHT = _net_get_tx_id(_ID_OFFSET_SERVO, 0)
    ARM_CENTER_RIGHT = _net_get_tx_id(_ID_OFFSET_SERVO, 1)
    ARM_CENTER = _net_get_tx_id(_ID_OFFSET_SERVO, 2)
    ARM_CENTER_LEFT = _net_get_tx_id(_ID_OFFSET_SERVO, 3)
    ARM_LEFT = _net_get_tx_id(_ID_OFFSET_SERVO, 4)
