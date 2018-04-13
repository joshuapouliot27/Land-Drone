import threading
import time


class Background_Thread(object):

    def __init__(self, function):
        self.function = function

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

    def run(self):
        self.function()