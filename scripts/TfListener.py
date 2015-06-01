#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
import Controller

if __name__ == '__main__':
    rospy.init_node('tf_listen')
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('marker_origin', 'desired_position_controller', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            continue

        print 'translation: ',trans
        angles = euler_from_quaternion(rot)
        print 'rotation: ',[(180.0/math.pi)*i for i in angles]

        rate.sleep()