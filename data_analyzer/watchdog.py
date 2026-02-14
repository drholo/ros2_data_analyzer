from rclpy.logging import get_logger
import threading
import time
import signal
import os


class WatchdogTimer:
    def __init__(self, timeout, timeout_handler=None):
        self.timeout = timeout
        self._timer = None
        self._lock = threading.Lock()
        self._stopped = False
        self._timeout_handler = timeout_handler
        self._logger = get_logger(__name__)

    def _start_timer(self):
        self._timer = threading.Timer(self.timeout, self._trigger)
        self._timer.start()

    def _trigger(self):
        with self._lock:
            if not self._stopped:
                self._logger.error("Watchdog expired! Sending CTRL+C...")
                self._logger.error(f"Process ID: {os.getpid()}")
                if self._timeout_handler:
                    self._timeout_handler()
                os.kill(os.getpid(), signal.SIGINT)

    def start(self):
        with self._lock:
            self._stopped = False
            self._start_timer()

    def ping(self):
        """Reset the timer."""
        with self._lock:
            if self._timer:
                self._timer.cancel()
            if not self._stopped:
                self._start_timer()

    def stop(self):
        with self._lock:
            self._stopped = True
            if self._timer:
                self._timer.cancel()


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
