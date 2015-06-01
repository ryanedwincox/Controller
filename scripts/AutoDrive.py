#!/usr/bin/env python

import time
import pygame
import rospy
import tf
from OrcusGUI import OrcusGUI
from Controller import *
from MotorController import *
from tf.transformations import euler_from_quaternion


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
    control.trans_x, control.trans_y, control.yaw, control.rise = 0, 0, 0, 0;
    control.trans_x_tare, control.trans_y_tare = 0, 0;
    control.yaw_tare, control.rise_tare = 0, 0;
    update_motor_values();
    write_motor_values(gui);

def update_joy_values(joystick, control):
    control.trans_x = joystick.get_axis(0);
    control.trans_y = -1 * joystick.get_axis(1);
    control.rise = -1 * joystick.get_axis(3);
    control.yaw = joystick.get_axis(2);

    #output_str = control + ",%s", rospy.get_time()
    #rospy.loginfo(output_str)

def process_joy_events():
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN and event.__dict__["button"] == 1:
            control.tare();
        if event.type == pygame.JOYBUTTONDOWN and event.__dict__["button"] == 0:
            if control.rise_control > -1:
                control.rise_control -= .05;
        if event.type == pygame.JOYBUTTONDOWN and event.__dict__["button"] == 3:
            if control.rise_control < 1:
                control.rise_control += .05;

def joy_init():
    """Initializes pygame and the joystick, and returns the joystick to be
    used."""

    pygame.init();
    pygame.joystick.init();
    if pygame.joystick.get_count() == 0:
        raise Exception("joy_init: No joysticks connected");
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    control.tare()
    
    return joystick

def normalize_0_1(value):
    PID_CONSTANT = 10 # temporary value
    value = value/PID_CONSTANT

    if value < -1:
        ctrl = 0
        ctrl_tare = 1
    elif -1 <= value < 0:
        ctrl = 0
        ctrl_tare = -value
    elif 0 <= value < 1:
        ctrl = value
        ctrl_tare = 0
    else: # value > 1
        ctrl = 1
        ctrl_tare = 0
    return (ctrl, ctrl_tare)

def update_controller(controller):
    # # check if a valid tf is being published and if not turn off motors
    # if (listener.frameExists('/camera')):
    #     print "camera tf being published"
    #     # control.trans_x, control.trans_y, control.rise = 0,0,0
    #     # x,y,z,yaw = 0,0,0,0
    # else:
    #     print "no camera tf being published"

    try:
        (trans,rot) = listener.lookupTransform('/desired_position', '/camera', rospy.Time(0))
        # (trans2,temp) = listener.lookupTransform('/camera', 'marker_origin', rospy.Time(0))
    
    except (tf.LookupException, tf.ConnectivityException):
        print "lookup Exception"
        control.trans_x, control.trans_y, control.rise = 0,0,0

    # measuredYaw = math.atan2(trans2[0], trans2[2])
    angles = euler_from_quaternion(rot) # in radians
    print "angles",str(angles)
    x,y,z,yaw = controller.update(trans,angles[1])
    # if (gui.resetMotorCommands):
    #     x,y,z,yaw = 0,0,0,0
    #     gui.RESETMOTORCMDS = False
    #print 'yaw command: ',yaw
    # these values need to be between 0 and 1
    #PID_CONSTANT = 10 # temporary value
    #control.trans_x = x / PID_CONSTANT 
    #control.trans_y = z / PID_CONSTANT
    #control.rise = y / PID_CONSTANT

    # # check if a valid tf is being published and if not turn off motors
    # if (listener.frameExists('/camera')):
    #     print "camera tf being published"
    #     # control.trans_x, control.trans_y, control.rise = 0,0,0
    #     # x,y,z,yaw = 0,0,0,0
    # else:
    #     print "no camera tf being published"

    control.rise,control.rise_tare = normalize_0_1(y)
    control.trans_x,control.trans_x_tare = normalize_0_1(x)
    control.trans_y,control.trans_y_tare = normalize_0_1(z)
    control.yaw,control.yaw_tare = normalize_0_1(yaw)

    #print "control: ",control
    #print "trans: ",trans


    

def mainDrive():

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

    process_joy_events()

    gui.after(1, mainDrive) # loops the mainDrive method


if __name__=="__main__":

    rospy.init_node('autodrive')

    # Start gui and call mainDrive loop
    gui = OrcusGUI()
    gui.master.geometry("800x550") # make sure all widgets start inside
    gui.master.minsize(800, 550)
    gui.master.maxsize(800, 550)
    gui.master.title('ROV ORCUS')

    joystick = joy_init()

    listener = tf.TransformListener()

    controller = Controller([0,0,0],0)

    gui.after(1, mainDrive)
    gui.mainloop()
