"""RS-family protocol handler for Robomow BLE."""

from __future__ import annotations

import logging
import struct
from asyncio import sleep as _async_sleep
from datetime import time
from time import monotonic
from typing import TYPE_CHECKING, Any

from .const import Message, MessageType, MowerOperatingState, MowerSchedule
from .const_rs import (
    GET_MESSAGE_PAYLOAD_SIZE,
    MISC_TYPE_MIN_SIZE,
    NO_MESSAGE_ID,
    get_stop_reason,
    SCHEDULE_DAY_DISABLED_MASK,
    SCHEDULE_DAY_FLAGS_OFFSET,
    SCHEDULE_DAYS_PER_WEEK,
    SCHEDULE_INACTIVE_END_OFFSET,
    SCHEDULE_INACTIVE_START_OFFSET,
    SCHEDULE_PAYLOAD_SIZE,
    SCHEDULE_WINDOW_END_OFFSET,
    SCHEDULE_WINDOW_START_OFFSET,
    STATE_ANTI_THEFT_ACTIVE_MASK,
    STATE_BATTERY_MASK,
    STATE_CHARGE_SOURCE_MASK,
    STATE_MOW_MOTOR_ACTIVE_MASK,
    STATE_FOLLOWING_WIRE_MASK,
    STATE_NEAR_BASE_MASK,
    MOWING_DEBOUNCE_SECONDS,
    STATE_PAYLOAD_SIZE,
    ZONE_ALL,
    DRIVE_BLADES_ON_FLAG,
    DRIVE_MAX_TICKS,
    DRIVE_TICK_SECONDS,
    MiscMessageType,
    OperationMode,
    RsMessageType,
)
from .const_rt import READ_EEPROM_PAYLOAD_SIZE
from .family_handler_base import RobomowFamilyHandler
from .helpers import check_payload_length

if TYPE_CHECKING:
    from .const import WireSignalType, Zone
    from .mower import PendingCommand

LOGGER = logging.getLogger(__package__)


class RobomowRsFamilyHandler(RobomowFamilyHandler):
    """RS-family Robomow BLE protocol behavior.

    RS-family mowers answer a smaller set of MISCELLANEOUS messages than RT
    mowers: STATE, GET_SCHEDULE and EXTENDED_STATE respond, while INFO,
    CONFIG_META_DATA, LAST_OPERATIONS and SUPPORTED_FEATURES return empty
    payloads. Several RT features therefore have no RS equivalent.
    """

    def __init__(self, device: Any) -> None:
        """Initialize the handler and its blade-debounce state."""
        super().__init__(device)
        self._mowing_until: float = 0.0
        self._drive_counter: int = 0
        self.eeprom_values: dict[int, int] = {}

    def _clear_mowing_debounce(self) -> None:
        """Forget the blade debounce so the next poll reports the real state."""
        self._mowing_until = 0.0

    async def async_initialize_state(self) -> None:
        """Initialize RS-family state after connection."""
        await self._device._async_send_misc_msg(MiscMessageType.GET_SCHEDULE)
        await self._device._async_send_misc_msg(MiscMessageType.STATE)

    async def async_poll_status(self) -> None:
        """Poll RS-family status while connected.

        GET_MESSAGE doubles as the keep-alive the mower expects; without
        periodic traffic it closes the link after roughly 15 seconds.
        """
        await self._device._async_send_msg(MessageType.GET_MESSAGE)
        await self._device._async_send_misc_msg(MiscMessageType.STATE)

    # --- Automatic operation commands ---

    async def _async_send_operation(self, mode: OperationMode) -> None:
        """Send an automatic-operation command."""
        self._clear_mowing_debounce()
        await self._device._async_send_msg_with_sequence(
            MessageType.COMMAND, struct.pack(">BB", int(mode), ZONE_ALL)
        )
        await self._device._async_send_misc_msg(MiscMessageType.STATE)

    async def async_start_mowing(
        self,
        duration_minutes: int | None = None,
        starting_zone: Zone | None = None,
    ) -> None:
        """Start mowing.

        RS-family mowers take neither a duration nor a starting zone on this
        command; both arguments are accepted for interface compatibility and
        ignored.
        """
        if duration_minutes is not None or starting_zone is not None:
            LOGGER.debug(
                "RS start_mowing ignores duration_minutes=%s and starting_zone=%s",
                duration_minutes,
                starting_zone,
            )
        await self._async_send_operation(OperationMode.MOW)

    async def async_start_mowing_edge(self) -> None:
        """Start edge mowing."""
        await self._async_send_operation(OperationMode.EDGE)

    async def async_stop_mowing(self) -> None:
        """Stop the current operation."""
        await self._async_send_operation(OperationMode.STOP)

    async def async_return_to_home(self) -> None:
        """Send the mower back to its base station."""
        await self._async_send_operation(OperationMode.BASE)

    async def async_drive(
        self,
        direction: int,
        speed: int = 100,
        ticks: int = 5,
        *,
        blades: bool = False,
    ) -> None:
        """Drive the mower manually for a bounded number of ticks.

        Each tick is DRIVE_TICK_SECONDS of movement. The mower halts by itself
        as soon as packets stop arriving, so a dropped link stops the machine
        rather than leaving it running.

        Args:
            direction: Steering value; negative turns one way, positive the
                other. The stock remote used -80 forward, 90 backward,
                -120 left and 35 right.
            speed: Drive speed, 0 to 100.
            ticks: Number of DRIVE_TICK_SECONDS packets to send.
            blades: Whether to run the blade motor while driving.
        """
        ticks = max(1, min(DRIVE_MAX_TICKS, ticks))
        speed = max(0, min(100, speed))
        direction &= 0xFF

        self._clear_mowing_debounce()
        LOGGER.debug(
            "RS drive: direction=%d speed=%d ticks=%d blades=%s",
            direction,
            speed,
            ticks,
            blades,
        )

        for _ in range(ticks):
            self._drive_counter = (self._drive_counter + 1) & 0x0F
            flags = (DRIVE_BLADES_ON_FLAG if blades else 0) | (
                self._drive_counter << 4
            )
            await self._device._async_send_msg(
                RsMessageType.DRIVE,
                bytes([flags, direction, speed, 0x00, 0x00]),
            )
            await _async_sleep(DRIVE_TICK_SECONDS)

        await self._device._async_send_misc_msg(MiscMessageType.STATE)

    # --- Unsupported settings ---

    def _unsupported(self, what: str) -> None:
        """Log a setting that has no known RS-family encoding."""
        LOGGER.warning("%s is not supported for RS-family mowers", what)

    async def async_enable_schedule(self) -> None:
        """Enable the mower schedule (not supported on RS)."""
        self._unsupported("Enabling the schedule")

    async def async_disable_schedule(self) -> None:
        """Disable the mower schedule (not supported on RS)."""
        self._unsupported("Disabling the schedule")

    async def async_set_schedule(self, schedule: MowerSchedule) -> None:
        """Set the mowing schedule (not supported on RS)."""
        self._unsupported("Setting the schedule")

    async def async_enable_anti_theft(self) -> None:
        """Enable anti-theft mode (not supported on RS)."""
        self._unsupported("Enabling anti-theft")

    async def async_disable_anti_theft(self) -> None:
        """Disable anti-theft mode (not supported on RS)."""
        self._unsupported("Disabling anti-theft")

    async def async_enable_child_lock(self) -> None:
        """Enable child lock (not supported on RS)."""
        self._unsupported("Enabling child lock")

    async def async_disable_child_lock(self) -> None:
        """Disable child lock (not supported on RS)."""
        self._unsupported("Disabling child lock")

    async def async_set_wire_signal_type(
        self, wire_signal_type: WireSignalType
    ) -> None:
        """Set the wire signal type (not supported on RS)."""
        self._unsupported("Setting the wire signal type")

    async def async_set_starting_point_a(self, value: int) -> None:
        """Set starting point A (not supported on RS)."""
        self._unsupported("Setting starting point A")

    async def async_set_starting_point_b(self, value: int) -> None:
        """Set starting point B (not supported on RS)."""
        self._unsupported("Setting starting point B")

    # --- Response handling ---

    def handle_get_message(self, payload: bytes | bytearray | memoryview) -> None:
        """Handle a GET_MESSAGE payload.

        The frame matches the RT layout, but RS-family mowers index both the
        message id and the stop id into the status text table rather than the
        separate message and error tables RT uses.

            [0]     message type flags
            [1] [2] message id, 0xFFFF when no message is active; describes
                    the current activity and indexes the message table
            [3] [4] stop id; the condition the mower displays, and indexes
                    the status text table
            [5] [6] failure id
        """
        if not check_payload_length(
            MessageType.GET_MESSAGE, payload, GET_MESSAGE_PAYLOAD_SIZE, exact=True
        ):
            return

        msg_flags, message_id, stop_id, failure_id = struct.unpack_from(">BHHH", payload)

        # The condition the mower displays comes from the stop id, resolved
        # against the RS stop reason table. The message id uses the same
        # numbering and is logged alongside it.
        message = Message(get_stop_reason(stop_id), number=stop_id)

        LOGGER.debug(
            "RS GET_MESSAGE: flags=0x%02X message_id=%d (%s) stop_id=%d failure_id=%d -> %s",
            msg_flags,
            message_id,
            get_stop_reason(message_id) if message_id != NO_MESSAGE_ID else "none",
            stop_id,
            failure_id,
            message,
        )
        self._device._set_message(message)

    def handle_read_eeprom_response(
        self, request: PendingCommand, response: PendingCommand
    ) -> None:
        """Handle a READ_EEPROM response.

        The RS parameter identifiers carry no known names, so values are
        recorded by numeric id in ``eeprom_values`` for later comparison
        rather than being mapped onto mower attributes.

        Request payload is N big-endian uint16 ids; the response is the
        matching N big-endian uint32 values, paired by position.
        """
        count = len(request.payload) // 2
        expected_size = READ_EEPROM_PAYLOAD_SIZE * count
        if not check_payload_length(
            MessageType.READ_EEPROM, response.payload, expected_size, exact=True
        ):
            LOGGER.warning(
                "RS READ_EEPROM response length %d, expected %d for %d ids: %s",
                len(response.payload),
                expected_size,
                count,
                bytes(response.payload).hex(),
            )
            return

        for index in range(count):
            param = struct.unpack_from(">H", request.payload, offset=index * 2)[0]
            value = struct.unpack_from(
                ">L", response.payload, offset=index * READ_EEPROM_PAYLOAD_SIZE
            )[0]
            self.eeprom_values[param] = value
            LOGGER.debug("  RS EEPROM: 0x%04X=0x%08X", param, value)

    def handle_miscellaneous_response(self, response: Any) -> None:
        """Handle a MISCELLANEOUS response after pending command matching."""
        if not check_payload_length(
            MessageType.MISCELLANEOUS, response.payload, MISC_TYPE_MIN_SIZE
        ):
            return

        raw_type = struct.unpack_from(">H", response.payload)[0]
        try:
            misc_type = MiscMessageType(raw_type)
        except ValueError:
            LOGGER.debug(
                "RS MISCELLANEOUS type 0x%02X not handled: %s",
                raw_type,
                bytes(response.payload).hex(),
            )
            return

        if misc_type is MiscMessageType.STATE:
            self._handle_misc_state(response.payload)
        elif misc_type is MiscMessageType.GET_SCHEDULE:
            self._handle_misc_schedule(response.payload)
        else:
            LOGGER.debug(
                "RS MISCELLANEOUS %s: %s",
                misc_type.name,
                bytes(response.payload).hex(),
            )

    def _handle_misc_state(self, payload: bytes | bytearray | memoryview) -> None:
        """Handle a STATE payload.

        Layout after the 2-byte type field, following RobotDataMiscellaneousRs:
            [0] status flags  bits 0-1 charge source, bit 2 following wire,
                              bit 4 near base, bit 5 mow motor active
            [1] operational state
            [2] battery       bits 0-6 percent, bit 7 anti-theft active
            [3] [4] unidentified 16-bit counter
            [5] [6] automatic operation duration in minutes
        """
        if not check_payload_length(
            MessageType.MISCELLANEOUS, payload, STATE_PAYLOAD_SIZE, exact=True
        ):
            return

        status_flags, _operational_state, battery = struct.unpack_from(
            ">BBB", payload, offset=MISC_TYPE_MIN_SIZE
        )

        charging = (status_flags & STATE_CHARGE_SOURCE_MASK) == 0
        blade_on = (status_flags & STATE_MOW_MOTOR_ACTIVE_MASK) != 0
        following_wire = (status_flags & STATE_FOLLOWING_WIRE_MASK) != 0
        near_base = (status_flags & STATE_NEAR_BASE_MASK) != 0

        now = monotonic()
        if blade_on:
            self._mowing_until = now + MOWING_DEBOUNCE_SECONDS
        mowing = blade_on or now < self._mowing_until

        if mowing:
            state = MowerOperatingState.MOWING
        elif following_wire and near_base:
            state = MowerOperatingState.RETURNING_HOME_FOLLOWING_EDGE
        elif following_wire:
            state = MowerOperatingState.GOING_TO_START
        elif charging:
            state = MowerOperatingState.CHARGING
        else:
            state = MowerOperatingState.IDLE

        self._device._set_state(state)
        self._device._set_battery_level(battery & STATE_BATTERY_MASK)
        self._device._set_anti_theft_active(
            (battery & STATE_ANTI_THEFT_ACTIVE_MASK) != 0
        )

    def _handle_misc_schedule(self, payload: bytes | bytearray | memoryview) -> None:
        """Handle a GET_SCHEDULE payload.

        Four time fields and a per-day mask are confirmed, each checked
        against a mower whose settings were known:

            [2:4]   flags; low 7 bits are the day mask (see below)
            [10:12] end of the daily mowing window
            [12:14] start of the daily mowing window
            [14:16] start of the inactive window
            [16:18] end of the inactive window

        Times are minutes past midnight. In the day mask, bit 0 is Monday
        through bit 6 Sunday, and a SET bit means that day is *disabled*.
        Offsets 4 to 9 have not been identified.
        """
        if not check_payload_length(
            MessageType.MISCELLANEOUS, payload, SCHEDULE_PAYLOAD_SIZE, exact=True
        ):
            return

        (day_flags,) = struct.unpack_from(
            ">H", payload, offset=SCHEDULE_DAY_FLAGS_OFFSET
        )
        (window_end,) = struct.unpack_from(
            ">H", payload, offset=SCHEDULE_WINDOW_END_OFFSET
        )
        (window_start,) = struct.unpack_from(
            ">H", payload, offset=SCHEDULE_WINDOW_START_OFFSET
        )
        (inactive_start,) = struct.unpack_from(
            ">H", payload, offset=SCHEDULE_INACTIVE_START_OFFSET
        )
        (inactive_end,) = struct.unpack_from(
            ">H", payload, offset=SCHEDULE_INACTIVE_END_OFFSET
        )

        disabled = day_flags & SCHEDULE_DAY_DISABLED_MASK
        days = tuple(
            MowerSchedule.Day(enabled=(disabled & (1 << index)) == 0)
            for index in range(SCHEDULE_DAYS_PER_WEEK)
        )

        schedule = MowerSchedule(
            start_time=time(hour=window_start // 60, minute=window_start % 60),
            end_time=time(hour=window_end // 60, minute=window_end % 60),
            day=days,
        )
        LOGGER.debug(
            "  RS GET_SCHEDULE: mow %02d:%02d-%02d:%02d inactive %02d:%02d-%02d:%02d "
            "flags=0x%04X enabled=%s",
            window_start // 60,
            window_start % 60,
            window_end // 60,
            window_end % 60,
            inactive_start // 60,
            inactive_start % 60,
            inactive_end // 60,
            inactive_end % 60,
            day_flags,
            [d.enabled for d in days],
        )
        self._device._set_schedule(schedule)
