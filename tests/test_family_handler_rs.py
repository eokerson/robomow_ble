"""Tests for the RS-family protocol handler.

The payloads below were captured from a Robomow RS-generation mower and the
decoded values verified against the Robomow phone app.
"""

from types import SimpleNamespace

from robomow_ble_lib.const import MowerOperatingState
from robomow_ble_lib.family_handler_rs import RobomowRsFamilyHandler
from robomow_ble_lib.mower import RobomowDevice


def _handler() -> tuple[RobomowRsFamilyHandler, RobomowDevice]:
    device = RobomowDevice(mainboard_serial="2411800002985", update_callback=None)
    return RobomowRsFamilyHandler(device), device


def _feed(handler: RobomowRsFamilyHandler, payload: bytes) -> None:
    handler.handle_miscellaneous_response(SimpleNamespace(payload=payload))


def test_state_while_mowing() -> None:
    """A mowing mower reports the mow motor active and a draining battery."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex("000b6a2b37201a003c"))

    assert device.operating_state == MowerOperatingState.MOWING
    assert device.battery_level == 55
    assert device.anti_theft_active is False


def test_state_while_charging() -> None:
    """A charging mower reports charge source active and is not mowing."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex("000b502a202041005f"))

    assert device.operating_state == MowerOperatingState.CHARGING
    assert device.battery_level == 32


def test_state_while_docked_not_charging() -> None:
    """A docked mower that is neither mowing nor charging reports idle."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex("000b522920203d005f"))

    assert device.operating_state == MowerOperatingState.IDLE
    assert device.battery_level == 32


def test_battery_masks_off_anti_theft_bit() -> None:
    """Bit 7 of the battery byte is the anti-theft flag, not part of the level."""
    handler, device = _handler()

    # battery byte 0xB7 = 0x37 (55 %) with the anti-theft bit set
    _feed(handler, bytes.fromhex("000b6a2bb7201a003c"))

    assert device.battery_level == 55
    assert device.anti_theft_active is True


def test_schedule_window_is_decoded() -> None:
    """The daily mowing window is stored as minutes past midnight."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex("000d014000555555050104b001e005640168"))

    assert device.schedule is not None
    assert device.schedule.start_time.hour == 8
    assert device.schedule.end_time.hour == 20


def test_unknown_misc_type_is_ignored() -> None:
    """An unhandled MISCELLANEOUS sub-type must not raise."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex("00080102"))

    assert device.battery_level is None


def test_short_state_payload_is_ignored() -> None:
    """A truncated STATE payload must not raise or set partial values."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex("000b6a2b"))

    assert device.battery_level is None


def test_state_while_returning_home() -> None:
    """Bit 2 of the status byte marks a mower driving back to its base."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex("000b562b61257f0001"))

    assert device.operating_state == MowerOperatingState.RETURNING_HOME_FOLLOWING_EDGE
    assert device.battery_level == 97
