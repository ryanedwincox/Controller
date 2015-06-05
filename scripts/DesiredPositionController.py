#!/usr/bin/env python
import rospy
import tf
import math
from sensor_msgs.msg import Joy

br = None
desired_trans = (0,0,0)
desired_rot = (0,math.pi,math.pi)

def callback(data):
    print "callback"
    global br
    global desired_trans
    global desired_rot

    trans = ( -0.01*data.axes[2],    # translation x axis
              0,                  # translation y axis
              -0.01*data.axes[3] )   # translation z axis

    desired_trans = map(sum, zip(trans, desired_trans))

    rot = (0, -math.pi/12*data.axes[0], 0)

    desired_rot = map(sum, zip(rot, desired_rot))

    br.sendTransform(desired_trans,
                tf.transformations.quaternion_from_euler(desired_rot[0],desired_rot[1],desired_rot[2]),
                rospy.Time.now(),
                "desired_position_controller",     # child
                "marker_origin"                    # parent
                )

def main():
    global br
    global desired_trans
    global desired_rot

    # starts the node
    rospy.init_node('DesiredPositionController')

    # broadcasting object
    br = tf.TransformBroadcaster()

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)

    # # run the node until it is destroyed
    # rospy.spin()
    
    # once a second, send desired so lookups don't time out
    # TODO: once per second may not be enough, adjust accordingly
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        br.sendTransform(desired_trans,
                tf.transformations.quaternion_from_euler(desired_rot[0],desired_rot[1],desired_rot[2]),
                rospy.Time.now(),
                "desired_position_controller",     # child
                "marker_origin"                    # parent
                )
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
