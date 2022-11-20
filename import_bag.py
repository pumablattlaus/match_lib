from typing import List, Callable, Tuple
import rospy
import rosbag
import os

# from match_lib.match_geometry import MyPose
from match_geometry import MyPose
from scipy.spatial.transform import Rotation
import cv2 as cv
from cv_bridge import CvBridge

import numpy as np
# rosrun tf tf_echo miranda/panda/panda_link7 miranda/panda/panda_link8

# TODO: base_footprint->base_link_inertia richtig herum?
# At time 0.000
# - Translation: [0.350, 0.150, 0.350]
# - Rotation: in Quaternion [0.000, 0.000, -0.000, 1.000]
#             in RPY (radian) [0.000, -0.000, -0.000]
#             in RPY (degree) [0.000, -0.000, -0.000]

def get_all_trees_from_bag(bag_file_name: str, frame_names: List[str], set_known_frames: Callable) -> Tuple[List[int], List[MyPose]]:
    pose_hand_fin = []
    pose_time = []
    tree = [None]*len(frame_names)
    set_known_frames(tree)
    for topic, msg, t in rosbag.Bag(bag_file_name).read_messages(topics=["/tf", "/mur/mir/robot_pose"]):
        if topic == "/mur/mir/robot_pose":
            tree[0]=MyPose(msg.position, msg.orientation)
        if topic == "/tf" and msg.transforms:
            for transf in msg.transforms:
                if transf.child_frame_id in frame_names:
                    # stamp_pose = transf.header.stamp
                    idx = frame_names.index(transf.child_frame_id)
                    tree[idx] = MyPose(transf.transform.translation.__reduce__()[2], transf.transform.rotation.__reduce__()[2])
                    
            if not None in tree:
                # Get link0 to end:
                pose_hand = MyPose()
                for p_trans in tree:
                    pose_hand += p_trans

                pose_hand_fin.append(pose_hand)
                pose_time.append(t)
                tree = [None]*len(frame_names)
                set_known_frames(tree)

    return pose_time, pose_hand_fin


def mur_write_all_trees_from_bag(bag_file_name: str):
    frame_names = ["robot_pose", "pose_to_base_link_inertia",
                "mur/ur/shoulder_link", 'mur/ur/upper_arm_link', 'mur/ur/forearm_link', 'mur/ur/wrist_1_link', 'mur/ur/wrist_2_link', 'mur/ur/wrist_3_link']

    pose_to_base_link_inertia = MyPose((-0.350, 0.150, -0.350), (0.000, 0.000, 0.000, 1.000))
    def set_known_frames(tree):
        tree[1]=pose_to_base_link_inertia
        return tree
                
    pose_time, pose_hand_fin = get_all_trees_from_bag(bag_file_name, frame_names, set_known_frames)
    with open(bag_file_name+'.csv','w') as fd:
        for t, pose in zip(pose_time, pose_hand_fin):
            fd.write(t.__str__()+";"+pose.position.__reduce__()[2].__str__()[1:-1] + ";" + pose.orientation.__reduce__()[2].__str__()[1:-1]+"\n")

def write_twists(bag_file_name, topic_name: str = "/mur/velocity_command"):
    twists = []
    times = []
    for topic, msg, t in rosbag.Bag(bag_file_name).read_messages(topics=[topic_name]):
        times.append(t)
        twists.append(msg)
    with open(bag_file_name+'_cmds.csv','w') as fd:
        for t, cmd in zip(times, twists):
            fd.write(t.__str__()+";"+cmd.linear.__reduce__()[2].__str__()[1:-1] + ";" + cmd.angular.__reduce__()[2].__str__()[1:-1]+"\n")


if __name__ == "__main__":
    path="/home/hauke/Documents/WindowsDocuments/"
    name = "2022-11-14-15-26-03.bag"
    bag_file_name=path+name

    # mur_write_all_trees_from_bag(bag_file_name)
    # write_twists(bag_file_name)

    