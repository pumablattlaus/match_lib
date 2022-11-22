#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs.msg
from match_lib.match_geometry import MyPose
from typing import List

class PosePublisher():

    def __init__(self, target_frames: List[str], source_frames: List[str], rate: float = 10.0, pub_topics: List[str] = None, tf_timeout=10):
        # check if node is already running
        if rospy.get_name() == "/unnamed":
            rospy.init_node("pose_publisher")
        if len(target_frames) != len(source_frames):
            raise ValueError("target_frames and source_frames must have the same length")
        if pub_topics is None:
            pub_topics = [f"/tf_pose_{source}_TO_{target}" for source, target in zip(source_frames, target_frames)]
        if len(pub_topics) != len(target_frames):
            raise ValueError("pub_topics must have the same length as target_frames")
        self.target_frames = target_frames
        self.source_frames = source_frames
        
        self.pubs = [rospy.Publisher(topic, geometry_msgs.msg.Pose, queue_size=10) for topic in pub_topics]
        self.listener = tf.TransformListener(tf_timeout)
        self.rate = rospy.Rate(rate)
        self.poses = [None] * len(target_frames)

    def publish_poses(self):
        while not rospy.is_shutdown():
            self.publish_poses_once()
            self.rate.sleep()
            
    def publish_poses_once(self):
        for pub, source, target in zip(self.pubs, self.source_frames, self.target_frames):
            try:
                t, r = self.listener.lookupTransform(source, target, rospy.Time(0))
                p = MyPose(t, r)
                pub.publish(p)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
    def get_poses(self):
        for i, (source, target) in enumerate(zip(self.source_frames, self.target_frames)):
            try:
                t, r = self.listener.lookupTransform(source, target, rospy.Time(0))
                p = MyPose(t, r)
                self.poses[i]=p
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return self.poses

if __name__ == '__main__':
    # source_frame = 'map'
    # rospy.init_node('tf_listener')

    target_frame = 'mur/ur/wrist_3_link'
    source_frame = 'mur/ur/base_link'
    source_frame_mir = "mur/mir/base_footprint"
    target_frame_mir = "map"
    source_frames = [source_frame, source_frame_mir]
    target_frames = [target_frame, target_frame_mir]

    pose_publisher = PosePublisher(target_frames, source_frames)
    pose_publisher.publish_poses()

