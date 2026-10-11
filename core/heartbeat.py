"""Background sender for the TriOS ground station heartbeat."""

from __future__ import annotations

import threading

from pymavlink import mavutil


class HeartbeatSender:
    """Send a MAVLink GCS heartbeat until stopped."""

    def __init__(self, master, rate_hz):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be greater than zero")

        self.master = master
        self.interval = 1 / rate_hz
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        """Start the sender thread, if it is not already running."""

        if self.is_running():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="TriOS-heartbeat",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        """Request shutdown and wait briefly for the sender thread."""

        thread = self._thread
        if thread is None:
            return

        self._stop_event.set()
        if thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.interval))

        if not thread.is_alive():
            self._thread = None

    def is_running(self):
        """Return whether the sender thread is currently alive."""

        return self._thread is not None and self._thread.is_alive()

    def _run(self):
        while not self._stop_event.is_set():
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
            except Exception as exc:
                print(f"Warning: failed to send GCS heartbeat: {exc}")

            self._stop_event.wait(self.interval)


if __name__ == "__main__":
    try:
        from .config import CONNECTION_STRING, HEARTBEAT_RATE_HZ, HEARTBEAT_TIMEOUT
        from .connection import close, connect
    except ImportError:  # Allow ``python core/heartbeat.py`` from the project root.
        from config import CONNECTION_STRING, HEARTBEAT_RATE_HZ, HEARTBEAT_TIMEOUT
        from connection import close, connect

    vehicle = connect(CONNECTION_STRING, HEARTBEAT_TIMEOUT)
    sender = HeartbeatSender(vehicle, HEARTBEAT_RATE_HZ)
    sender.start()
    try:
        print("Sending GCS heartbeats for 10 seconds")
        threading.Event().wait(10)
    finally:
        sender.stop()
        close(vehicle)
