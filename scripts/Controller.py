#!/usr/bin/env python

import math
import rospy
from PID import *
from MotorController import Control
from std_msgs.msg import String

# import note for the math: 
#   - z is the axis pointing out of the dock
#   - y is height
#   - x is right left
#   - r is the radius from the dock
#   - gamma is the angle of the ROV relative to the dock

class Controller():
    def __init__(self, trans, yaw):

        x,y,z = trans

        # control:
        # 1. the radius from the dock,
        # 2. the height of the ROV relative to the dock,
        # 3. the angle of the ROV relative to the dock (between pi and -pi)
        # self.desired = ( math.sqrt(x**2 + z**2), y, math.atan2(x, z) )
        self.desired = (x, y, z)
        self.desiredYaw = yaw

        #            Angle of the rov       Height of the ROV      Radius from the dock   Yaw?
        self.pid = ( PID(Kp=0, Kd=0, Ki=0), PID(Kp=0, Kd=0, Ki=0), PID(Kp=0, Kd=0, Ki=0), PID(Kp=2, Kd=0, Ki=0) )

        # make sure TunerGUI.py is running before running this
        rospy.Subscriber("pidConstants", String, self.callback)

    def setDesired(self, trans, yaw):
        x,y,z = trans
        # self.desired = ( math.sqrt(x**2 + z**2), y, math.atan2(x, z) )
        self.desired = trans
        self.desiredYaw = yaw

    def update(self, trans, yaw):
        x,y,z = trans
        # print 'measured yaw is: ',yaw
        transN = ( math.sqrt(x**2 + z**2), y, math.atan2(x, z) )

        error = [desired_i - trans_i for desired_i, trans_i in zip(self.desired, trans)] #transN for r,y,gamma
        errorYaw = self.desiredYaw - yaw

        # these are in r y gamma format and we need to go back to x y z
        r,y,gamma = [ self.pid[i].update(error[i]) for i in range(3) ]
        yawCommand = self.pid[3].update(errorYaw)

        # return [ r * math.cos(gamma), y, r * math.sin(gamma) ]
        return [r,y,gamma,yawCommand]

    def setPidParameters(self, parameters):
        [ self.pid[i].setParameters(parameters[i*3:(i+1)*3]) for i in range(4) ]

    def callback(self, data):
        # use bytes to convert data from ROS string to python string
        # if you use python 3, this will become a problem
        pidParameters = bytes(data)[7:].split(',')
        self.setPidParameters([ float(i) for i in pidParameters ])

    
