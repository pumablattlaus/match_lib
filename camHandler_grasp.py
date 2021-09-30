#!/usr/bin/env python
# coding=latin-1

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import std_msgs.msg
import geometry_msgs.msg as geom_msg 

import time
import copy
import tf
import tf2_ros

from match_geometry import MyPoint

class CameraHandler(object):
    """Handler to interact with Camera from other node
    """
    def __init__(self, listener=None, toFrame="map"):
        self.pub_runCam = rospy.Publisher("/cam/run", std_msgs.msg.Bool, queue_size=1)
        self.subGripP = rospy.Subscriber("/points_grasp", PointCloud2, self.cb_graspP, queue_size=10)
        self.graspP = []

        self.listener = listener
        self.toFrame = toFrame

    def run(self, boolVal=True):
        """Starts/Stops gripPoints detection of camera script

        Args:
            boolVal (bool, optional): Starts cam if True, else stops. Defaults to True.
        """
        msg = std_msgs.msg.Bool()
        msg.data = boolVal
        self.pub_runCam.publish(msg)

        if not boolVal: self.graspP = [] #reset

    def cb_graspP(self, msg=PointCloud2()):
        assert isinstance(msg, PointCloud2)
        
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.graspP = []
        for p in gen:
            if self.listener is not None: #transform points to map-coordinates 
                p_msg = geom_msg.PointStamped()
                p_msg.header = msg.header
                p_msg.point.x, p_msg.point.y, p_msg.point.z = p
                p_msg = self.listener.transformPoint(self.toFrame, p_msg)
                p = p_msg.point
            self.graspP.append(MyPoint(p))

    def getGraspP(self):
        """Starts/Stops detection and returns the 2 graspPoints (transformed to "toFrame")

        Returns:
            list(MyPoint): Grasp points from edge detection
        """
        while len(self.graspP) != 2:
            self.run()
            time.sleep(0.5)
        graspP = copy.copy(self.graspP)
        self.run(False)
        return graspP

if __name__ == '__main__':
    
    rospy.init_node("TEST_cam_otherNode")
    time.sleep(1)

    tf_caster = tf2_ros.StaticTransformBroadcaster()
    trans=(1.0,0.0,0.0)
    rot=(0.0,0.0,0.0,1.0)
    tf_msg = geom_msg.TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "map"
    tf_msg.child_frame_id = "camera_depth_optical_frame"
    tf_msg.transform.translation.x = trans[0]
    tf_msg.transform.translation.y = trans[1]
    tf_msg.transform.translation.z = trans[2]
    tf_msg.transform.rotation.x = rot[0]
    tf_msg.transform.rotation.y = rot[1]
    tf_msg.transform.rotation.z = rot[2]
    tf_msg.transform.rotation.w = rot[3]
    tf_caster.sendTransform(tf_msg)
    

    listener = tf.TransformListener()

    cam=CameraHandler(listener, "map")
    time.sleep(1)
    points = cam.getGraspP()

    print(points)