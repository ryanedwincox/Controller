import roslib
import rospy
import tf

PI = 3.1415926

if __name__ == '__main__':
    rospy.init_node('navigate')
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    step = 0

    stepsTrans = ((0.0, 0.0, 1.0),
                  (0.0, 0.0, 0.5))

    stepsRot = (tf.transformations.quaternion_from_euler(PI, 0, 0), 
                tf.transformations.quaternion_from_euler(PI, 0, 0))
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        broadcaster.sendTransform(stepsTrans[step],
                                        stepsRot[step],
                                        rospy.Time.now(),
                                        "desired_position",
                                        "marker_origin")

        broadcaster.sendTransform((0,0,0),
                                        (0,0,0,1),
                                        rospy.Time.now(),
                                        "world_origin",
                                        "world")
        try:
            (cameraTrans,cameraRot) = listener.lookupTransform('/camera', '/marker_origin', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # check if camera is close to desired_position and if so move to next step
        # note: for now only looks at trans
        if (abs(stepsTrans[step][0] - cameraTrans[0]) < 0.1 and 
            abs(stepsTrans[step][1] - cameraTrans[1]) < 0.1 and 
            abs(stepsTrans[step][2] - cameraTrans[2]) < 0.1):

            print "reached desired position"
            if (step + 1 < len(stepsTrans)):
                step = step + 1