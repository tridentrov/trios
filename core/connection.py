"""Helpers for opening and closing a TriOS MAVLink connection."""

from __future__ import annotations

import time

from pymavlink import mavutil

try:
    from . import config
    from .telemetry import Telemetry, wait_for_vehicle_heartbeat
except ImportError:  # Allow ``python core/connection.py`` from the project root.
    import config  # type: ignore[no-redef]
    from telemetry import Telemetry, wait_for_vehicle_heartbeat  # type: ignore[no-redef]


def wait_for_heartbeat(master, timeout):
    """Wait for and identify the vehicle heartbeat on a new link."""

    return wait_for_vehicle_heartbeat(master, timeout)


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
    telemetry = Telemetry(master)
    telemetry.start()
    try:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            time.sleep(min(1, remaining))
            if telemetry.link_alive():
                print(
                    f"Heartbeat: mode={telemetry.get_mode()}, "
                    f"armed={telemetry.is_armed()}"
                )
    finally:
        telemetry.stop()


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
