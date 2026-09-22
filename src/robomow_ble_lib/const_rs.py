"""Constants for the RS-family Robomow BLE protocol."""

from __future__ import annotations

from enum import IntEnum
from types import MappingProxyType

# A MISCELLANEOUS payload starts with a 2-byte type field.
MISC_TYPE_MIN_SIZE = 2

# Total payload size of a STATE response: 2 type bytes + 7 data bytes.
STATE_PAYLOAD_SIZE = 9

# Total payload size of a GET_SCHEDULE response: 2 type bytes + 16 data bytes.
SCHEDULE_PAYLOAD_SIZE = 18

# GET_SCHEDULE field offsets, measured against the payload including the two
# subtype bytes. Established by comparing captures against a mower whose
# settings were known, and by toggling one day and diffing the result.
SCHEDULE_DAY_FLAGS_OFFSET = 2
SCHEDULE_WINDOW_END_OFFSET = 10
SCHEDULE_WINDOW_START_OFFSET = 12
SCHEDULE_INACTIVE_START_OFFSET = 14
SCHEDULE_INACTIVE_END_OFFSET = 16

# Low seven bits of the flags field are a per-day mask, bit 0 Monday through
# bit 6 Sunday. A SET bit means that day is DISABLED, which is the opposite
# of the sense the name suggests.
SCHEDULE_DAY_DISABLED_MASK = 0x7F
SCHEDULE_DAYS_PER_WEEK = 7


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

# Stop reason codes, from the "Operation Stop Reason" table in the RS service
# guide. The same numbers appear in the stop id field of GET_MESSAGE.
#
# The guide gives each code an LCD message and a separate description of the
# cause. For 22 of the 77 codes the LCD message is literally "No message" —
# the mower stops and displays nothing — so those entries carry the guide's
# description instead. Reporting "No message" would be faithful to the screen
# and useless to the reader.
#
# Where the guide says "STOP button", RS models without one (the 612p among
# them) use the pull handle for the same function.
STOP_REASONS: MappingProxyType[int, str] = MappingProxyType(
    {
        0: "N/A",
        1: "STOP handle pulled",
        2: "No wire signal",
        3: "Start inside",
        4: "Key pressed",
        5: "Bumper pressed",
        6: "Front wheel prob.",
        7: "Stuck in place/Cross",
        8: "Stuck in place",
        9: "Check power",
        10: "Charging halted — no charging voltage",
        11: "Stopped by One Time Setup — base position or wire test ended",
        12: "Stopped by One Time Setup — obstacle event",
        13: "Drive overheat",
        14: "Base problem",
        15: "Recharge battery",
        16: "Drive overheat",
        17: "Recharge battery",
        18: "Recharge battery",
        19: "Charging halted — charger overheat",
        20: "Carrying handle lifted",
        21: "Handle lifted",
        22: "Start elsewhere",
        23: "Start elsewhere",
        24: "Stuck in place",
        25: "Switch off before lifting",
        26: "System switch turned off",
        27: "Mow overheat",
        28: "Check mow height",
        29: "Check mow height",
        30: "No wire signal",
        31: "Mow overheat",
        32: "Cross outside",
        33: "Front wheel prob.",
        34: "Inactive Time",
        35: "Time Completed",
        36: "Rain detected",
        37: "BIT edge terminate test — end of edge detected",
        38: "Remote control safety button pressed",
        39: "STOP handle pulled during manual operation",
        40: "Rain detected",
        41: "Front wheel drop-off too long — sent back to base",
        42: "Front wheel prob.",
        43: "Bumper held too long — sent back to base",
        44: "Bumper pressed",
        45: "Drive overheat",
        46: "Mow overheat",
        47: "No wire signal",
        48: "N/A",
        49: "Wrong menu place detected",
        50: "Recharge battery",
        51: "Recharge battery",
        52: "Recharge battery",
        53: "Recharge battery",
        54: "Recharge battery",
        55: "Recharge battery",
        56: "BIT near-wire test — end of edge detected",
        57: "UP/DOWN/Cancel button held — panic mode",
        59: "Low Temperature",
        60: "Check mow height",
        61: "Stuck in place",
        62: "Bumper pressed",
        63: "Stuck in place",
        64: "Stuck in place",
        65: "Floater Problem",
        66: "Base problem",
        67: "Docking station detected while going to entry point",
        68: "Battery overheat while charging",
        69: "UP button held — panic mode",
        70: "DOWN button held — panic mode",
        71: "Stop command received from the mobile app",
        72: "Start elsewhere",
        73: "Charging stopped to send a GSM message",
        74: "Drive overheat",
        75: "Drive overheat",
        76: "Rain detected",
        77: "Battery capacity timeout",
    }
)


def get_stop_reason(number: int) -> str:
    """Look up the text the mower displays for a stop reason code."""
    return STOP_REASONS.get(number, f"Unknown stop reason {number}")


class RsMessageType(IntEnum):
    """RS message types that have no entry in the shared MessageType enum."""

    DRIVE = 0x1A


# Manual drive. The mower stops on its own once packets stop arriving, so a
# move is expressed as a number of ticks at this interval.
DRIVE_TICK_SECONDS = 0.2
DRIVE_MAX_TICKS = 50
DRIVE_BLADES_ON_FLAG = 0x02
