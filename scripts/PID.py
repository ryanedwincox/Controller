#!/usr/bin/env python

import time

class PID():

    def __init__(self, Kp=0, Kd=0, Ki=0, Integrator_max=500, Integrator_min=-500):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min

        self.previous_error = 0

        # results stored in these variables
        self.P = 0
        self.D = 0
        self.I = 0

    def setParameters(self, parameters):
        Kp,Ki,Kd = parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def update(self, error):

        self.P = self.Kp * error

        self.I += error
        if self.I > self.Integrator_max:
            self.I = self.Integrator_max
        elif self.I < self.Integrator_min:
            self.I = self.Integrator_min

        self.D = error - self.previous_error
        self.previous_error = error

        return self.P + (self.Kd * self.D) + (self.Ki * self.I)

    def reset(self):
        self.P = 0
        self.D = 0
        self.I = 0

if __name__=="__main__":
    pid = PID(Kp=2, Kd=50, Ki=0)

    desired = 0
    actual = 0
    print "desired, %s" % desired
    while actual != 4:
        # get measurement
        print "actual, %s" % actual

        # update from PID controller
        control = pid.update(1)

        # send control value to the ROV
        print "control, %s" % control