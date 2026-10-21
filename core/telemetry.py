"""Receive MAVLink telemetry and retain the latest vehicle state."""

from __future__ import annotations

import threading
import time

from pymavlink import mavutil

try:
    from .config import HEARTBEAT_LOST_AFTER
except ImportError:  # Allow ``python core/telemetry.py`` from the project root.
    from config import HEARTBEAT_LOST_AFTER  # type: ignore[no-redef]


def wait_for_vehicle_heartbeat(master, timeout):
    """Wait for the vehicle heartbeat used to identify a MAVLink link.

    Connection setup has to identify the target before :class:`Telemetry` can
    filter messages.  Keeping this initial receive here also leaves this
    module as the single application-level reader of incoming messages.
    """

    deadline = time.monotonic() + timeout
    gcs_type = mavutil.mavlink.MAV_TYPE_GCS

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                f"No autopilot heartbeat received within {timeout} seconds"
            )

        heartbeat = master.recv_match(
            type="HEARTBEAT", blocking=True, timeout=remaining
        )
        if heartbeat is None:
            raise TimeoutError(
                f"No autopilot heartbeat received within {timeout} seconds"
            )

        if getattr(heartbeat, "type", None) == gcs_type:
            continue

        source_system = heartbeat.get_srcSystem()
        source_component = heartbeat.get_srcComponent()
        if not (1 <= source_system <= 255 and 1 <= source_component <= 255):
            raise ValueError(
                "Received an autopilot heartbeat with invalid target IDs: "
                f"system={source_system}, component={source_component}"
            )

        master.target_system = source_system
        master.target_component = source_component
        print(
            "Connected to autopilot "
            f"system {master.target_system}, component {master.target_component}"
        )
        return heartbeat


def request_streams(master, rate_hz):
    """Ask the vehicle to send all MAVLink data streams at ``rate_hz``."""

    if rate_hz <= 0:
        raise ValueError("rate_hz must be greater than zero")

    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1,
    )


class Telemetry:
    """Read MAVLink messages in one thread and expose the latest vehicle state."""

    def __init__(self, master):
        self.master = master
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

        self._attitude = None
        self._depth = None
        self._battery = None
        self._mode = None
        self._armed = False
        self._last_heartbeat = None

        # mode_mapping() is a vehicle-specific lookup.  Read it once so the
        # receive loop never has to call into the master while handling data.
        mode_mapping = getattr(master, "mode_mapping", lambda: {})() or {}
        self._mode_names = {number: name for name, number in mode_mapping.items()}

    def start(self):
        """Start the telemetry reader if it is not already running."""

        if self.is_running():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="TriOS-telemetry",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        """Request shutdown and wait for the reader to leave its receive call."""

        thread = self._thread
        if thread is None:
            return

        self._stop_event.set()
        if thread is not threading.current_thread():
            thread.join(timeout=1.1)

        if not thread.is_alive():
            self._thread = None

    def is_running(self):
        """Return whether the telemetry reader thread is alive."""

        return self._thread is not None and self._thread.is_alive()

    def _run(self):
        while not self._stop_event.is_set():
            try:
                message = self.master.recv_match(blocking=True, timeout=1)
                if message is None:
                    continue

                if self._message_type(message) == "BAD_DATA":
                    continue

                self._handle(message)
            except Exception as exc:
                # A malformed message or a transient link error must not end
                # telemetry.  Give the stop event a chance before retrying so
                # a persistent error does not create a busy loop.
                if self._stop_event.wait(0.1):
                    break
                print(f"Warning: failed to receive telemetry: {exc}")

    def _handle(self, message):
        message_type = self._message_type(message)
        if message_type == "BAD_DATA":
            return

        with self._lock:
            if message_type == "HEARTBEAT":
                # The GCS heartbeat is on the same link.  Only a heartbeat
                # from the system selected by connect() describes the vehicle.
                if message.get_srcSystem() != self.master.target_system:
                    return

                self._armed = bool(
                    message.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self._mode = self._mode_names.get(
                    message.custom_mode, message.custom_mode
                )
                self._last_heartbeat = time.monotonic()
            elif message_type == "ATTITUDE":
                self._attitude = {
                    "roll": message.roll,
                    "pitch": message.pitch,
                    "yaw": message.yaw,
                }
            elif message_type == "VFR_HUD":
                # ArduSub reports depth as a negative altitude.  Keep the
                # MAVLink value unchanged until the vehicle convention is
                # confirmed by SITL.
                self._depth = message.alt
            elif message_type == "SYS_STATUS":
                self._battery = {
                    "voltage": message.voltage_battery / 1000.0,
                    "current": message.current_battery / 100.0,
                    "remaining": message.battery_remaining,
                }

    @staticmethod
    def _message_type(message):
        get_type = getattr(message, "get_type", None)
        if callable(get_type):
            return get_type()
        return getattr(message, "_type", getattr(message, "type", None))

    @property
    def attitude(self):
        """A snapshot of the latest attitude."""

        return self.get_attitude()

    @property
    def depth(self):
        """The latest raw VFR_HUD altitude used as depth."""

        return self.get_depth()

    @property
    def battery(self):
        """A snapshot of the latest battery values."""

        return self.get_battery()

    @property
    def mode(self):
        """The latest vehicle mode."""

        return self.get_mode()

    @property
    def armed(self):
        """The latest vehicle armed state."""

        return self.is_armed()

    def get_attitude(self):
        """Return a snapshot of roll, pitch, and yaw."""

        with self._lock:
            return None if self._attitude is None else self._attitude.copy()

    def get_depth(self):
        """Return the latest raw VFR_HUD altitude used as depth."""

        with self._lock:
            return self._depth

    def get_battery(self):
        """Return a snapshot of battery voltage (V), current (A), and percent."""

        with self._lock:
            return None if self._battery is None else self._battery.copy()

    def get_mode(self):
        """Return the latest vehicle mode name or custom mode value."""

        with self._lock:
            return self._mode

    def is_armed(self):
        """Return the latest vehicle armed state."""

        with self._lock:
            return self._armed

    def heartbeat_age(self):
        """Return seconds since the last vehicle heartbeat.

        Infinity is returned before the first vehicle heartbeat, which makes
        the link correctly report as unavailable until one is received.
        """

        with self._lock:
            last_heartbeat = self._last_heartbeat

        if last_heartbeat is None:
            return float("inf")
        return max(0.0, time.monotonic() - last_heartbeat)

    def link_alive(self):
        """Return whether a recent vehicle heartbeat has been received."""

        return self.heartbeat_age() < HEARTBEAT_LOST_AFTER
