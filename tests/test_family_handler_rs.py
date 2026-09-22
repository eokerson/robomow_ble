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


# Three GET_SCHEDULE frames captured from the mower's wired toolkit port while
# the Robomow phone app toggled one day. The schedule went Sunday-off, then
# Saturday-and-Sunday-off, then back, so the same bytes are seen twice with a
# different state in between.
_SCHEDULE_SUNDAY_OFF = "000d014000555555050104b001e005640168"
_SCHEDULE_SAT_SUN_OFF = "000d016000555555050104b001e005640168"


def test_schedule_marks_sunday_disabled() -> None:
    """Bit 6 of the day mask is Sunday, and a set bit means disabled."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex(_SCHEDULE_SUNDAY_OFF))

    assert device.schedule is not None
    enabled = [day.enabled for day in device.schedule.day]
    assert enabled == [True, True, True, True, True, True, False]


def test_schedule_tracks_a_second_disabled_day() -> None:
    """Disabling Saturday sets bit 5 while Sunday's bit 6 stays set."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex(_SCHEDULE_SAT_SUN_OFF))

    assert device.schedule is not None
    enabled = [day.enabled for day in device.schedule.day]
    assert enabled == [True, True, True, True, True, False, False]


def test_schedule_day_mask_round_trips() -> None:
    """Re-enabling the day restores the original decode exactly."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex(_SCHEDULE_SAT_SUN_OFF))
    _feed(handler, bytes.fromhex(_SCHEDULE_SUNDAY_OFF))

    assert device.schedule is not None
    enabled = [day.enabled for day in device.schedule.day]
    assert enabled == [True, True, True, True, True, True, False]


def test_schedule_window_survives_a_day_change() -> None:
    """Toggling a day must not disturb the mowing window fields."""
    handler, device = _handler()

    _feed(handler, bytes.fromhex(_SCHEDULE_SAT_SUN_OFF))

    assert device.schedule is not None
    assert device.schedule.start_time.hour == 8
    assert device.schedule.end_time.hour == 20


def test_stop_reason_reports_the_cause_not_a_blank_lcd() -> None:
    """Codes whose LCD message is blank must still say why the mower stopped.

    Stop reason 20 is the one that prompted this: the mower halts, the screen
    shows nothing, and the guide's description is the only thing that explains
    it. Verified against a live stop after the handle was pulled.
    """
    handler, device = _handler()

    _feed_msg(handler, bytes.fromhex("00ffff00140000"))

    assert device.message is not None
    text = device.message.title
    assert "No message" not in text
    assert "handle" in text.lower()


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
    assert "Bumper pressed" in str(device.message)


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


def test_drive_builds_bounded_frames() -> None:
    """Manual drive sends one frame per tick with a rolling safety counter."""
    import asyncio

    handler, device = _handler()
    sent: list[tuple[int, bytes]] = []

    async def fake_send(msg_type, payload=None):
        sent.append((int(msg_type), bytes(payload or b"")))
        return True

    device._async_send_msg = fake_send          # type: ignore[assignment]
    device._async_send_misc_msg = fake_send     # type: ignore[assignment]

    asyncio.run(handler.async_drive(direction=-80, speed=100, ticks=3))

    drives = [p for t, p in sent if t == 0x1A]
    assert len(drives) == 3
    for payload in drives:
        assert len(payload) == 5
        assert payload[1] == 0xB0      # -80 as an unsigned byte
        assert payload[2] == 100       # speed
        assert payload[0] & 0x02 == 0  # blades off by default
    # the safety counter must advance between packets
    assert len({p[0] >> 4 for p in drives}) == 3


def test_drive_clamps_ticks_and_speed() -> None:
    """Out-of-range arguments are clamped rather than sent verbatim."""
    import asyncio

    from robomow_ble_lib.const_rs import DRIVE_MAX_TICKS

    handler, device = _handler()
    sent: list[tuple[int, bytes]] = []

    async def fake_send(msg_type, payload=None):
        sent.append((int(msg_type), bytes(payload or b"")))
        return True

    device._async_send_msg = fake_send          # type: ignore[assignment]
    device._async_send_misc_msg = fake_send     # type: ignore[assignment]

    asyncio.run(handler.async_drive(direction=0, speed=999, ticks=9999))

    drives = [p for t, p in sent if t == 0x1A]
    assert len(drives) == DRIVE_MAX_TICKS
    assert all(p[2] == 100 for p in drives)
