#!/usr/bin/env python3
# # coding=latin-1

import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped,Pose
from gazebo_msgs.msg import ModelStates

class GazeboPose:
    def __init__(self, model_name='mur216'):
        """Gets the current pose of the model and calculates the velocity

        Args:
            model_name (str, optional): Defaults to 'mur216'.
        """        
        self.gazebo_pose_stamped = None
        self.gazebo_velocity_stamped = None
        self.gazebo_velocity_stamped_filtered = None
        self.model_name = model_name
    
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_gazebo_pose)

    def cb_gazebo_pose(self, msg):
        """get pose and velocity from gazebo"""
        gazebo_pose = msg.pose[msg.name.index(self.model_name)]
        gazebo_pose_stamped = PoseStamped()
        gazebo_pose_stamped.pose.pose = gazebo_pose
        gazebo_pose_stamped.header.stamp = rospy.Time.now()
        gazebo_velocity_stamped = self.get_velocity_from_poses(self.gazebo_pose_stamped, gazebo_pose_stamped)
        if gazebo_velocity_stamped is not None:
            self.gazebo_velocity_stamped = gazebo_velocity_stamped
            self.gazebo_velocity_stamped_filtered = gazebo_velocity_stamped # no filtering for now
        self.gazebo_pose_stamped = gazebo_pose_stamped

    def get_velocity_from_poses(self, pose_old=PoseStamped(), pose_new=PoseStamped()):
        """get velocity from two poses"""

        if pose_old is None or pose_new is None:
            rospy.loginfo("pose_old or pose_new is None")
            return

        position_old = np.array([pose_old.pose.pose.position.x, pose_old.pose.pose.position.y, pose_old.pose.pose.position.z])
        position_new = np.array([pose_new.pose.pose.position.x, pose_new.pose.pose.position.y, pose_new.pose.pose.position.z])
        orientation_old = np.array([pose_old.pose.pose.orientation.x, pose_old.pose.pose.orientation.y, pose_old.pose.pose.orientation.z, pose_old.pose.pose.orientation.w])
        orientation_new = np.array([pose_new.pose.pose.orientation.x, pose_new.pose.pose.orientation.y, pose_new.pose.pose.orientation.z, pose_new.pose.pose.orientation.w])
        # quaternion_old = transformations.quaternion_matrix(orientation_old)
        # quaternion_new = transformations.quaternion_matrix(orientation_new)

        delta_t=pose_new.header.stamp.secs-pose_old.header.stamp.secs
        if delta_t==0:
            return

        velocity_lin = (position_new - position_old) / delta_t
        velocity_ang = (orientation_old - orientation_new) / delta_t

        velocity = TwistStamped()
        velocity.twist.linear.x = velocity_lin[0]
        velocity.twist.linear.y = velocity_lin[1]
        velocity.twist.linear.z = velocity_lin[2]
        velocity.twist.angular.x = velocity_ang[0]
        velocity.twist.angular.y = velocity_ang[1]
        velocity.twist.angular.z = velocity_ang[2]
        velocity.header.stamp = pose_new.header.stamp

        return velocity

class GazeboVelocity:
    def __init__(self, model_name='mur216'):
        """Gets the current velocity of the model

        Args:
            model_name (str, optional): Defaults to 'mur216'.
        """        
        self.gazebo_velocity_stamped = None
        self.gazebo_velocity_stamped_filtered = None
        self.model_name = model_name
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_gazebo_velocity)

    def cb_gazebo_velocity(self, msg):
        """get velocity from gazebo"""
        gazebo_velocity = msg.twist[msg.name.index(self.model_name)]
        gazebo_velocity_stamped = TwistStamped(header=Header(stamp=rospy.Time.now()), twist=gazebo_velocity)
        self.gazebo_velocity_stamped = gazebo_velocity_stamped
        self.gazebo_velocity_stamped_filtered = gazebo_velocity_stamped # no filtering for now

if __name__ == '__main__':
    rospy.init_node('gazebo_pose')
    gazebo_pose = GazeboPose()
    gazebo_velocity = GazeboVelocity()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print(gazebo_pose.gazebo_pose_stamped)
        print(gazebo_velocity.gazebo_velocity_stamped)
        rate.sleep()