from rclpy.logging import get_logger
from threading import Event, Lock, Thread
from typing import TYPE_CHECKING, Callable
import time
import signal
import os

if TYPE_CHECKING:
    from rclpy.impl.rcutils_logger import RcutilsLogger


class WatchdogTimer:
    timeout: float
    _thread: Thread = None
    _lock: Lock
    _stop_event: Event
    _stopped: bool
    _last_ping: float
    _timeout_handler: Callable
    _send_signal: bool
    _logger: "RcutilsLogger"

    def __init__(self, timeout, timeout_handler=None, send_signal: bool = True):
        self.timeout = timeout
        self._lock = Lock()
        self._stop_event = Event()
        self._stopped = False
        self._last_ping = time.monotonic()
        self._timeout_handler = timeout_handler
        self._send_signal = send_signal
        self._logger = get_logger(__name__)

    def _run(self):
        while True:
            if self._stop_event.wait(timeout=0.2):
                return
            with self._lock:
                if self._stopped:
                    return
                since_ping = time.monotonic() - self._last_ping
                expired = since_ping > self.timeout
            if expired:
                self._trigger()
                return

    def _trigger(self):
        handler = None
        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            self._stop_event.set()
            handler = self._timeout_handler
        self._logger.error(
            f"Watchdog expired! Last ping {time.monotonic() - self._last_ping:.3f}s ago."
            + (" Sending CTRL+C..." if self._send_signal else "")
        )
        self._logger.error(f"Process ID: {os.getpid()}")
        if handler:
            handler()
        if self._send_signal:
            os.kill(os.getpid(), signal.SIGINT)

    def start(self):
        with self._lock:
            self._stopped = False
            self._stop_event.clear()
            self._last_ping = time.monotonic()
            if not self._thread or not self._thread.is_alive():
                self._thread = Thread(target=self._run, daemon=True)
                self._thread.start()

    def set_timeout_handler(self, timeout_handler: Callable | None) -> None:
        with self._lock:
            self._timeout_handler = timeout_handler

    def ping(self):
        """Reset the timer."""
        with self._lock:
            if not self._stopped:
                self._last_ping = time.monotonic()
                self._logger.debug("Watchdog ping")

    def stop(self):
        with self._lock:
            self._stopped = True
            self._stop_event.set()


if __name__ == "__main__":

    def save_everything():
        print("Saving all data before shutdown...")

    # Example usage
    watchdog = WatchdogTimer(timeout=5, timeout_handler=save_everything)
    watchdog.start()

    try:
        for _ in range(4):
            time.sleep(1)
            print("Ping watchdog")
            watchdog.ping()
    except KeyboardInterrupt:
        print("Program interrupted by watchdog or user. Exiting...")
        watchdog.stop()
