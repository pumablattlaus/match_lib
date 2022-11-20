#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs.msg
from match_geometry import MyPose

if __name__ == '__main__':
    target_frame = 'mur/ur/wrist_3_link'
    source_frame = 'mur/ur/base_link'
    
    source_frame_mir = "mur/mir/base_footprint"
    target_frame_mir = "map"

    # source_frame = 'map'
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    pub = rospy.Publisher('tf_pose', geometry_msgs.msg.Pose,queue_size=1)
    pub_mir = rospy.Publisher('tf_pose_mir', geometry_msgs.msg.Pose,queue_size=1)

    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        try:
            # listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            pose = MyPose(trans, rot)
            pub.publish(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            pass
        try:
            # listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
            (trans_mir,rot_mir) = listener.lookupTransform(target_frame_mir, source_frame_mir, rospy.Time(0))
            pose_mir = MyPose(trans_mir, rot_mir)
            pub_mir.publish(pose_mir)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            pass



        rate.sleep()