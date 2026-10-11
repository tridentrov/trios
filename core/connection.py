"""Helpers for opening and closing a TriOS MAVLink connection."""

from __future__ import annotations

import time

from pymavlink import mavutil

try:
    from . import config
except ImportError:  # Allow ``python core/connection.py`` from the project root.
    import config  # type: ignore[no-redef]


def wait_for_heartbeat(master, timeout):
    """Wait for an autopilot heartbeat and return the received message.

    GCS heartbeats are ignored because they do not identify the vehicle TriOS
    needs to control.  A timeout is reported explicitly as a connection error.
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

        # recv_match normally sets these through pymavlink's message handler;
        # assign them explicitly so the contract remains true for every link.
        master.target_system = source_system
        master.target_component = source_component
        print(
            "Connected to autopilot "
            f"system {master.target_system}, component {master.target_component}"
        )
        return heartbeat


def connect(connection_string, timeout):
    """Open a MAVLink link, wait for the vehicle heartbeat, and return it."""

    master = mavutil.mavlink_connection(
        connection_string,
        source_system=config.SOURCE_SYSTEM,
    )
    try:
        wait_for_heartbeat(master, timeout)
    except Exception:
        close(master)
        raise
    return master


def close(master):
    """Close a MAVLink link cleanly."""

    if master is not None:
        master.close()


def _print_heartbeats(master, duration=5):
    """Print vehicle mode and armed state for a short diagnostic run."""

    deadline = time.monotonic() + duration
    gcs_type = mavutil.mavlink.MAV_TYPE_GCS
    armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        heartbeat = master.recv_match(
            type="HEARTBEAT", blocking=True, timeout=min(1, remaining)
        )
        if heartbeat is None or getattr(heartbeat, "type", None) == gcs_type:
            continue

        mode = mavutil.mode_string_v10(heartbeat)
        armed = bool(heartbeat.base_mode & armed_flag)
        print(f"Heartbeat: mode={mode}, armed={armed}")


if __name__ == "__main__":
    vehicle = connect(config.CONNECTION_STRING, config.HEARTBEAT_TIMEOUT)
    try:
        print(
            f"Target system: {vehicle.target_system}, "
            f"target component: {vehicle.target_component}"
        )
        _print_heartbeats(vehicle)
    finally:
        close(vehicle)
