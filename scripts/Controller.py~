import from PID *

class zControl():
    def __init__(self, desired=0):
        self.desired = desired
        self.pid = PID(Kp=2, Kd=3, Ki=0.5)

    def setDesired(self, desired):
        self.desired = desired

    def update(self, current_value):
        error = desired - current_value
        return pid.update(error)

    def sendToRov(self):
        # something