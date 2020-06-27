"""
Network configuration module.
Includes every bus slave and its sub data.
The enum BoardID includes all the base board IDs used in the design.
The NED_ADDRESS* lists define the hardware CAN addresses for ISOTP protocol.
"""

from enum import IntEnum

from highlevel.adapter.socket.isotp import ISOTPAddress

""" " " " " " " " " " " " " " " " " 
Utilities for this file.
""" " " " " " " " " " " " " " " " "


def _net_get_tx_id(id_offset: int, index: int) -> int:
    """
    Returns the transmission id of a board, given its type's start_id and its index.
    """
    return 2 * (id_offset + index)


def _net_get_rx_id(id_offset: int, index: int) -> int:
    """
    Returns the reception id of a board, given its type's start_id and its index.
    """
    return _net_get_tx_id(id_offset, index) + 1


_CAN_DEVICE = "can0"

ID_OFFSET_MOTOR = 0x00
ID_OFFSET_SERVO = 0x10

""" " " " " " " " " " " " " " " " " 
ISOTP ADDRESS OBJECTS FOR BOARDS
""" " " " " " " " " " " " " " " " "
NB_SERVO_BOARDS = 7

NET_ADDRESS_MOTOR = ISOTPAddress(_CAN_DEVICE, 1, 0)
NET_ADDRESSES_SERVO = [
    ISOTPAddress(_CAN_DEVICE,
                 _net_get_rx_id(ID_OFFSET_SERVO, i),
                 _net_get_tx_id(ID_OFFSET_SERVO, i)
                 )
    for i in range(0, NB_SERVO_BOARDS)
]

""" " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " "
Base Board IDs.
Used to determine the CAN addresses used to transmit to/receive from them.
""" " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " " "


class BoardIDs(IntEnum):
    """
    Board IDs
    """
    # MOTOR BOARD
    MOTOR = ID_OFFSET_MOTOR + 0x00

    # Actuator/servo boards
    ARM_RIGHT = ID_OFFSET_SERVO + 0x00
    ARM_CENTER_RIGHT = ID_OFFSET_SERVO + 0x01
    ARM_CENTER = ID_OFFSET_SERVO + 0x02
    ARM_CENTER_LEFT = ID_OFFSET_SERVO + 0x03
    ARM_LEFT = ID_OFFSET_SERVO + 0x04
