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


def test_going_to_start_when_following_wire_away_from_base() -> None:
    """Bit 2 set with bit 4 clear means the mower is heading out, not home."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex("000b462b63257f0000"))

    assert device.operating_state == MowerOperatingState.GOING_TO_START


def test_blade_off_transient_is_debounced() -> None:
    """A single blade-off sample mid-run must not flap the state to idle."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex("000b6a2b63257f0001"))
    assert device.operating_state == MowerOperatingState.MOWING

    # blade motor off while the mower reverses at the boundary
    _feed(handler, bytes.fromhex("000b4a2b62257e0003"))
    assert device.operating_state == MowerOperatingState.MOWING

    _feed(handler, bytes.fromhex("000b6a2b62257e0003"))
    assert device.operating_state == MowerOperatingState.MOWING


def test_blade_off_beyond_debounce_reports_real_state() -> None:
    """Once the debounce expires the true state is reported."""
    import robomow_ble_lib.family_handler_rs as mod

    handler, device = _handler()
    _feed(handler, bytes.fromhex("000b6a2b63257f0001"))
    assert device.operating_state == MowerOperatingState.MOWING

    handler._mowing_until = 0.0  # simulate the debounce window elapsing
    _feed(handler, bytes.fromhex("000b502a62257e0006"))

    assert device.operating_state == MowerOperatingState.CHARGING


def test_command_clears_the_debounce() -> None:
    """Issuing a command must not leave a stale MOWING debounce in place."""
    handler, _device = _handler()
    handler._mowing_until = 1e12

    handler._clear_mowing_debounce()

    assert handler._mowing_until == 0.0


def _feed_msg(handler: RobomowRsFamilyHandler, payload: bytes) -> None:
    handler.handle_get_message(payload)


def test_get_message_reports_an_active_fault() -> None:
    """An active message id is resolved against the status text table."""
    handler, device = _handler()

    # captured when the mower halted outside the perimeter wire
    _feed_msg(handler, bytes.fromhex("050020002c0000"))

    assert device.message is not None
    assert "Stuck on the wire" in str(device.message)


def test_get_message_falls_back_to_the_stop_reason() -> None:
    """With no active message the stop id describes why the run ended."""
    handler, device = _handler()

    _feed_msg(handler, bytes.fromhex("00ffff00230000"))

    assert device.message is not None
    assert "Time Completed" in str(device.message)


def test_get_message_ignores_a_short_payload() -> None:
    """A truncated GET_MESSAGE payload must not raise."""
    handler, device = _handler()

    _feed_msg(handler, bytes.fromhex("00ffff"))

    assert device.message is None



def test_get_message_matches_the_mower_display() -> None:
    """The stop id carries the condition shown on the mower's own screen."""
    handler, device = _handler()

    # captured while the mower displayed "Start inside" after being driven
    # outside the perimeter wire
    _feed_msg(handler, bytes.fromhex("050019000300 00".replace(" ", "")))

    assert device.message is not None
    assert "Start inside" in str(device.message)
