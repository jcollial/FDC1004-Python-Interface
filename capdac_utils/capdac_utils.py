import signal


class ShutdownHandler:
    def __init__(self):
        self._shutdown_requested = False
        signal.signal(signal.SIGINT, self._handle_sigint)

    def _handle_sigint(self, signum, frame):
        print("SIGINT received. Preparing to shut down...")
        self._shutdown_requested = True

    def check_shutdown(self):
        return self._shutdown_requested
