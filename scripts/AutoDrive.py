#!/usr/bin/env python

import time
import pygame
import rospy
import tf
from OrcusGUI import OrcusGUI
from Controller import *
from MotorController import *
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool


"""
      Front

 FR_LF     FR_RT
   /---------\
  /|         |\
   |  FR_VT  |
   |    o    |
   |         |
   |    o    |
   |  BA_VT  |
  \|         |/
   \---------/
 BA_LF     BA_RT

"""

control = Control()


def onexit():
    print "exiting"
    turnOffMotors()
    update_motor_values();
    write_motor_values(gui);

def update_joy_values(joystick, control):
    # control.trans_x = joystick.get_axis(0);
    # control.trans_y = -1 * joystick.get_axis(1);
    # # control.rise = -1 * joystick.get_axis(3);
    # control.yaw = joystick.get_axis(2);
    # control.rise = -(joystick.get_axis(5) + 1) / 2 + (joystick.get_axis(4) + 1) / 2
    # print "LT: ", str(joystick.get_axis(5) + 1)
    # print "RT: ", str(joystick.get_axis(4) + 1)

    #output_str = control + ",%s", rospy.get_time()
    #rospy.loginfo(output_str)

def process_joy_events(joystick):
    # define button numbers
    A = 0
    X = 2
    Y = 3
    B = 1
    LB = 4 
    RB = 5 
    # get events
    for event in pygame.event.get():
        # process button presses
        if event.type == pygame.JOYBUTTONDOWN:
            if joystick.get_button(A):
                control.tare();
                # print "joystick tare"
            if joystick.get_button(B):
                # print "Button B****************************************************************************************"
                pass
            if joystick.get_button(X):
                # print "Button X**** ************************************************************************************"
                pass
            if joystick.get_button(Y):
                # print "Button Y****************************************************************************************"
                pass
            if joystick.get_button(LB):
                if control.rise_control > -1:
                    control.rise_control -= .05;
                    # print "increase rise tare"
            if joystick.get_button(RB):
                if control.rise_control < 1:
                    control.rise_control += .05;
                    # print "decrease rise tare"

def joy_init():
    """Initializes pygame and the joystick, and returns the joystick to be
    used."""

    pygame.init();
    pygame.joystick.init();
    if pygame.joystick.get_count() == 0:
        raise Exception("joy_init: No joysticks connected");
    joystick = pygame.joystick.Joystick(2)
    joystick.init()
    
    control.tare()
    
    return joystick

def normalize_neg1_to_1(value):
    PID_CONSTANT = 10 # not sure where this value comes from
    value = value/PID_CONSTANT

    # limit the value to between -1 and 1
    if value < -1:
        value = -1
    elif value > 1:
        value = 1

    return value

def positionStatusCallback(data):
    global position_known
    position_known = data.data

def turnOffMotors():
    control.rise = 0
    control.trans_x = 0
    control.trans_y = 0
    control.yaw = 0
    control.rise_tare = 0
    control.trans_x_tare = 0
    control.trans_y_tare = 0
    control.yaw_tare = 0

def update_controller(controller):
    # if position_known:
    #     print "position known"
    try:
        (trans,rot) = listener.lookupTransform('/desired_position', '/camera', rospy.Time(0))
        (trans2,temp) = listener.lookupTransform('/marker_origin', '/camera', rospy.Time(0))

        # calculate angle from the ROV to the dock
        angleToDock = math.atan2(trans2[0], trans2[2])

        # change the desired yaw so that the camera will try to face the dock
        controller.setDesiredYaw(angleToDock)

        # angles from the rot matrix
        rotAngles = euler_from_quaternion(rot) # in radians

        # update the controller by sending the error
        x,y,z,yaw = controller.update(trans,rotAngles[1])

        # print "x,y,z,yaw: ", str(x), ", ", str(y) , ", ", str(z) , ", ",str(yaw)

        control.rise = normalize_neg1_to_1(y)
        control.trans_x = normalize_neg1_to_1(x)
        control.trans_y = normalize_neg1_to_1(z)
        control.yaw = normalize_neg1_to_1(yaw)
        control.rise_tare = 0
        control.trans_x_tare = 0
        control.trans_y_tare = 0
        control.yaw_tare = 0
    
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "Exception"
        # turn off motors if there is an exception
        turnOffMotors()

    # else:
    #     print "position unknown"
    #     # turn off motors if position unknown
    #     turnOffMotors()


def mainDrive():
    print 'mainDrive'
    if (gui.clickedNavButton):
        turnOffMotors()
        gui.resetNavButton()

    if (gui.navigateStatus()):
        update_controller(controller)
    else:
        pass
        update_joy_values(joystick, control)

    update_motor_values(control)

    # sets the motor values to the sliders
    # motors[MOTOR.FR_LF].power = gui.frontLeft * 1.25
    # motors[MOTOR.FR_RT].power = gui.frontRight * 1.25
    # motors[MOTOR.BA_LF].power = gui.backLeft * 1.25 
    # motors[MOTOR.BA_RT].power = gui.backRight * 1.25
    # motors[MOTOR.FR_VT].power = gui.frontVert * 1.25
    # motors[MOTOR.BA_VT].power = gui.backVert * 1.25

    gui.drawMotorStatus(motors)
    gui.estopControl()

    write_motor_values(gui)

    process_joy_events(joystick)

    gui.after(1, mainDrive) # loops the mainDrive method


if __name__=="__main__":

    rospy.init_node('autodrive')

    rospy.Subscriber("position_known", Bool, positionStatusCallback)

    # Start gui and call mainDrive loop
    gui = OrcusGUI()
    gui.master.geometry("812x550") # make sure all widgets start inside
    gui.master.minsize(812, 550)
    gui.master.maxsize(812, 550)
    gui.master.title('ROV ORCUS')

    joystick = joy_init()

    listener = tf.TransformListener()

    controller = Controller([0,0,0],0)

    gui.after(1, mainDrive)
    gui.mainloop()
