#!/usr/bin/env python

import math
import pygame
from MOTOR import MOTOR
from serial import Serial, SerialException, SerialTimeoutException

ser = None

def connect(port_name):
    """Returns a Serial object that is connected to the port_name. Returns
    None if the connection could not be made."""

    global ser

    try:
        ser = Serial(port_name, timeout = None, writeTimeout = 0.5)
    except SerialException:
        print("connect: could not connect to port " + port_name)

class Motor:
    def __init__(self, ID, pow_header, dir_header, power = 0):
        self.ID = ID
        self.pow_header = pow_header
        self.dir_header = dir_header
        self.power = power

motors = {MOTOR.FR_LF: Motor(MOTOR.FR_LF, b'1', b'a'),
          MOTOR.FR_RT: Motor(MOTOR.FR_RT, b'2', b'b'),
          MOTOR.BA_RT: Motor(MOTOR.BA_RT, b'3', b'c'),
          MOTOR.BA_LF: Motor(MOTOR.BA_LF, b'4', b'd'),
          MOTOR.FR_VT: Motor(MOTOR.FR_VT, b'5', b'e'),
          MOTOR.BA_VT: Motor(MOTOR.BA_VT, b'6', b'f')}

# values in range [0, 1]
class Control:
    def __init__(self, trans_x = 0, trans_y = 0, yaw = 0, rise = 0,
                 trans_x_tare = 0, trans_y_tare = 0, yaw_tare = 0, rise_tare = 0,
                 rise_control = 0):
        self.trans_x = trans_x;
        self.trans_y = trans_y;
        self.yaw = yaw;
        self.rise = rise;
        self.trans_x_tare = trans_x_tare;
        self.trans_y_tare = trans_y_tare;
        self.yaw_tare = yaw_tare;
        self.rise_tare = rise_tare;
        self.rise_control = rise_control;

    def __str__(self):
        return '{self.trans_x},{self.trans_y},{self.yaw},{self.rise},{self.trans_x_tare},{self.trans_y_tare},{self.yaw_tare},{self.rise_tare},{self.rise_control}'.format(self=self)

    def tare(self):
        self.trans_x_tare = self.trans_x;
        self.trans_y_tare = self.trans_y;
        self.yaw_tare = self.yaw;
        self.rise_tare = self.rise;

    def trans_x_value(self):
        return self.trans_x - self.trans_x_tare;

    def trans_y_value(self):
        return self.trans_y - self.trans_y_tare;

    def yaw_value(self):
        return self.yaw - self.yaw_tare;

    def rise_value(self):
        return self.rise - self.rise_tare + self.rise_control;

def write_motor_values(gui):
    global ser

    if (ser == None):
        connect("/dev/ttyACM0")

    write_timeout = False
    ser.flushOutput()
    ser.flushInput()
    try:
        write_motor_values_helper(ser)
    except SerialTimeoutException:
        print "write timeout"
        write_timeout = True

    # creates new serial if write attempts are failing
    if write_timeout == True:
        gui.navigateCallback()
        ser.close()
        connect("/dev/ttyACM0")


def write_motor_values_helper(ser):
	"""Writes the motor power and direction to the serial port.
	Each write consists of a header followed by the value repeated twice."""

	# do something about this
	global motors

	if not isinstance(ser, Serial):
	    raise ValueError("write_motor_values: ser not of type Serial, of type",
	                     type(ser));


	for motor in motors:
	    pow = motors[motor].power        
	    power_bytes = bytearray()
	    power_bytes.append(int(abs(pow)))
	    power_bytes *= 2
	    power_bytes.insert(0, motors[motor].pow_header)
	    ser.write(power_bytes)

	    dir = b'1' if pow > 0 else b'0'
	    dir_bytes = bytearray()
	    dir_bytes.append(dir)
	    dir_bytes *= 2
	    dir_bytes.insert(0, motors[motor].dir_header)
	    ser.write(dir_bytes)

def update_motor_values(control):
    # do something about thisself.
    global motors;

    for motor in motors:
        motors[motor].power = get_motor_power(motor, control);

def get_motor_power(n, control):
    """Takes the motor number and returns the magnitude of the total power it
    should output, from -255 to 255. Returns 0 if an invalid motor is given."""
    
    power_sum = get_trans_power(n, control) + get_yaw_power(n, control) + \
                get_rise_power(n, control);
    scaled_power = int(power_sum * 255);

    try:
        return min(scaled_power, 255) if scaled_power > 0 else max(scaled_power, -255);
    except ValueError as bad_motor:
        print(bad_motor.args[0]);
        return 0;


def get_trans_power(n, control):
    """Takes the motor number and returns the power it should output for
    translational motion, from -1 to 1.
    
    Raises a ValueError if the motor number is unrecognized."""
    
    # these motors don't have an effect on translational speed
    if n == MOTOR.FR_VT or n == MOTOR.BA_VT:
        return 0;
        
    x = control.trans_x_value();
    y = control.trans_y_value();
    if x == 0 and y == 0:
        return 0;
    
    m1 = .5 * x + y / (2 * math.sqrt(3));
    m2 = -.5 * x + y / (2 * math.sqrt(3));
    m1_norm = m1 / abs(max(m1, m2)) * min(math.hypot(x, y), 1);
    m2_norm = m2 / abs(max(m1, m2)) *  min(math.hypot(x, y), 1);
    if n == MOTOR.FR_LF:
        return -1 * m1_norm;
    if n == MOTOR.FR_RT:
        return -1 * m2_norm;
    if n == MOTOR.BA_RT:
        return m1_norm;
    if n == MOTOR.BA_LF:
        return m2_norm;
    raise ValueError("get_trans_power: Illegal motor number");


def get_yaw_power(n, control):
    """Takes the motor number and returns the power it should output for
    rotational motion, from -1 to 1.
    
    Raises a ValueError if the motor number is unrecognized."""

    # these motors don't have an effect on translational speed
    if n == MOTOR.FR_VT or n == MOTOR.BA_VT:
        return 0;
    if n == MOTOR.FR_LF or n == MOTOR.BA_RT:
        return -.4 *  control.yaw_value();
    if n == MOTOR.FR_RT or n == MOTOR.BA_LF:
        return .4 * control.yaw_value();
    raise ValueError("get_yaw_power: Illegal motor number");


def get_rise_power(n, control):
    """Takes the motor number and returns the power it should output for
    vertical motion, from -1 to 1.
    
    Raises a ValueError if the motor number is unrecognized."""

    # these motors don't have an effect on translational speed
    if n == MOTOR.FR_LF or n == MOTOR.BA_RT or \
       n == MOTOR.FR_RT or n == MOTOR.BA_LF:
        return 0;
    if n == MOTOR.FR_VT or n == MOTOR.BA_VT:
        return control.rise_value();
    raise ValueError("get_rise_power: Illegal motor number")