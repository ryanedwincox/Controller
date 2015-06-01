#!/usr/bin/env python
import rospy
import tf
import math
from sensor_msgs.msg import Joy

br = None
desired_trans = (0,0,0)
desired_rot = (0,0,0,0)

def callback(data):
    global br
    global desired_trans
    global desired_rot

    trans = ( -0.01*data.axes[2],    # translation x axis
              0,                  # translation y axis
              -0.01*data.axes[3] )   # translation z axis

    desired_trans = map(sum, zip(trans, desired_trans))

    rot = tf.transformations.quaternion_from_euler( 
            0,    # rotation x axis
            -20*data.axes[0],    # rotation y axis
            0 )   # rotation z axis 

    desired_rot = map(sum, zip(rot, desired_rot))

    br.sendTransform(desired_trans,
                desired_rot,
                rospy.Time.now(),
                "desired_position_controller",     # child
                "marker_origin"                    # parent
                )

def main():
    global br

    # starts the node
    rospy.init_node('DesiredPositionController')

    # broadcasting object
    br = tf.TransformBroadcaster()

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)

    # run the node until it is destroyed
    rospy.spin()

if __name__ == '__main__':
    main()