import threading
import time


class Background_Thread(object):

    def __init__(self, function, arguments=()):
        self.function = function
        self.arguments = arguments

        thread = threading.Thread(target=self.run, args=arguments)
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

    def run(self):
        self.function(*self.arguments)
