"""RS-family protocol handler for Robomow BLE."""

from __future__ import annotations

import logging
import struct
from datetime import time
from time import monotonic
from typing import TYPE_CHECKING, Any

from .const import Message, MessageType, MowerOperatingState, MowerSchedule
from .const_rs import (
    GET_MESSAGE_PAYLOAD_SIZE,
    MISC_TYPE_MIN_SIZE,
    NO_MESSAGE_ID,
    get_stop_reason,
    SCHEDULE_PAYLOAD_SIZE,
    STATE_ANTI_THEFT_ACTIVE_MASK,
    STATE_BATTERY_MASK,
    STATE_CHARGE_SOURCE_MASK,
    STATE_MOW_MOTOR_ACTIVE_MASK,
    STATE_FOLLOWING_WIRE_MASK,
    STATE_NEAR_BASE_MASK,
    MOWING_DEBOUNCE_SECONDS,
    STATE_PAYLOAD_SIZE,
    ZONE_ALL,
    MiscMessageType,
    OperationMode,
)
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

        The RS EEPROM parameter identifiers are not known, so responses are
        only logged.
        """
        LOGGER.debug(
            "RS READ_EEPROM: %s => %s",
            bytes(request.payload).hex(),
            bytes(response.payload).hex(),
        )

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

        The two confirmed fields are the daily mowing window, stored as
        minutes past midnight at offsets 10 and 12.
        """
        if not check_payload_length(
            MessageType.MISCELLANEOUS, payload, SCHEDULE_PAYLOAD_SIZE, exact=True
        ):
            return

        window_end, window_start = struct.unpack_from(">HH", payload, offset=10)

        schedule = MowerSchedule(
            start_time=time(hour=window_start // 60, minute=window_start % 60),
            end_time=time(hour=window_end // 60, minute=window_end % 60),
        )
        self._device._set_schedule(schedule)
