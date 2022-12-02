#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Vector3Stamped
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

class RotatePosePublisher():

    def __init__(self, target_frame, source_frame, topic_sub, topic_pub, tf_timeout=10):
        # super().__init__(tf_timeout) # if we want to inherit from TfListener
        # check if node is already running
        if rospy.get_name() == "/unnamed":
            rospy.init_node("pose_publisher")

        self.listener = tf.TransformListener(tf_timeout)
        self.target_frame = target_frame
        self.source_frame = source_frame

        self.pub = rospy.Publisher(topic_pub, Pose, queue_size=10)
        self.sub = rospy.Subscriber(topic_sub, Pose, self.callback)
        self.vector = Vector3Stamped()
        self.vector.header.frame_id = self.source_frame


    def callback(self, msg: Pose):
        if msg is None:
            return
        self.vector.header.stamp = rospy.Time()

        msg.position = self.rotate_xyz(msg.position)
        msg.orientation = self.rotate_xyz(msg.orientation)
        self.pub.publish(msg)


    def rotate_xyz(self, msg):
        self.vector.vector.x = msg.x
        self.vector.vector.y = msg.y
        self.vector.vector.z = msg.z
        try:
            vector_out = self.listener.transformVector3(self.target_frame, self.vector)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(e)
            return
        msg.x, msg.y, msg.z = vector_out.vector.x, vector_out.vector.y, vector_out.vector.z
        return msg

        

if __name__ == '__main__':
    # source_frame = 'map'
    # rospy.init_node('tf_listener')

    rospy.set_param('/use_sim_time', True)

    target_frame = 'mur/ur/wrist_3_link'
    source_frame = 'mur/ur/base_link'
    source_frame_mir = "mur/mir/base_footprint"
    target_frame_mir = "map"
    source_frames = [source_frame, source_frame_mir]
    target_frames = [target_frame, target_frame_mir]

    pose_publisher = PosePublisher(target_frames, source_frames)
    rotPos = RotatePosePublisher("/map", pose_publisher.source_frames[0], pose_publisher.pubs[0].name, "/mur/ur/target_pose_rotated")
    pose_publisher.publish_poses()

