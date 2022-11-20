#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs.msg
from match_geometry import MyPose

if __name__ == '__main__':
    target_frame = 'mur/ur/wrist_3_link'
    source_frame = 'map'
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    pub = rospy.Publisher('tf_pose', geometry_msgs.msg.Pose,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            continue

        pose = MyPose(trans, rot)
        pub.publish(pose)

        rate.sleep()