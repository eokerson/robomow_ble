"""Constants for the RS-family Robomow BLE protocol."""

from __future__ import annotations

from enum import IntEnum

# A MISCELLANEOUS payload starts with a 2-byte type field.
MISC_TYPE_MIN_SIZE = 2

# Total payload size of a STATE response: 2 type bytes + 7 data bytes.
STATE_PAYLOAD_SIZE = 9

# Total payload size of a GET_SCHEDULE response: 2 type bytes + 16 data bytes.
SCHEDULE_PAYLOAD_SIZE = 18


class MiscMessageType(IntEnum):
    """RS miscellaneous message sub-types that the mower answers."""

    STATE = 0x0B
    GET_SCHEDULE = 0x0D
    EXTENDED_STATE = 0x27


class OperationMode(IntEnum):
    """Operation modes accepted by the automatic-operation command."""

    STOP = 0
    EDGE = 1
    MOW = 2
    BASE = 3


# Zone selector used by the automatic-operation command; 0xFF is main/all zones.
ZONE_ALL = 0xFF

# STATE byte 8 (status flags).
STATE_CHARGE_SOURCE_MASK = 0x03
STATE_MOW_MOTOR_ACTIVE_MASK = 0x20
# Bit 2 marks the mower following the perimeter wire; bit 4 distinguishes
# homebound (set) from outbound (clear).
STATE_FOLLOWING_WIRE_MASK = 0x04
STATE_NEAR_BASE_MASK = 0x10

# The blade motor stops briefly whenever the mower reverses and turns at the
# boundary. Observed gaps are a single poll sample (2 s), so MOWING is held for
# this long after the last blade-on reading to stop the state flapping.
MOWING_DEBOUNCE_SECONDS = 10.0

# STATE byte 10 (battery).
STATE_BATTERY_MASK = 0x7F
STATE_ANTI_THEFT_ACTIVE_MASK = 0x80

# Total payload size of a GET_MESSAGE response.
GET_MESSAGE_PAYLOAD_SIZE = 7

# Sentinel written to the message id field when no message is active.
NO_MESSAGE_ID = 0xFFFF
