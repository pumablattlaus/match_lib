#!/usr/bin/env python
# coding=latin-1

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import std_msgs.msg
import time
import copy

class CameraHandler(object):
    """Handler to interact with Camera from other node
    """
    def __init__(self):
        self.pub_runCam = rospy.Publisher("/cam/run", std_msgs.msg.Bool, queue_size=1)
        self.subGripP = rospy.Subscriber("/points_grasp", PointCloud2, self.cb_graspP, queue_size=10)
        self.grasP = []

    def run(self, boolVal=True):
        """Starts/Stops gripPoints detection of camera script

        Args:
            boolVal (bool, optional): Starts cam if True, else stops. Defaults to True.
        """
        msg = std_msgs.msg.Bool()
        msg.data = boolVal
        self.pub_runCam.publish(msg)

        if not boolVal: self.grasP = [] #reset

    def cb_graspP(self, msg=PointCloud2()):
        assert isinstance(msg, PointCloud2)
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.grasP = []
        for p in gen:
            self.grasP.append(p)

    def getGraspP(self):
        """Starts/Stops detection and returns the 2 graspPoints

        Returns:
            list: Grasp points from edge detection
        """
        while len(self.grasP) != 2:
            self.run()
            time.sleep(0.5)
        graspP = copy.copy(self.grasP)
        self.run(False)
        return graspP

if __name__ == '__main__':
    
    rospy.init_node("TEST_cam_otherNode")
    time.sleep(1)
    cam=CameraHandler()
    time.sleep(1)
    points = cam.getGraspP()

    print(points)