import threading


class Background_Thread(object):

    def __init__(self, , arguments=None):
        self.function = function
        self.arguments = arguments

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

    def run(self):
        if self.arguments is not None:
            self.function(*self.arguments)
        else:
            self.function()
