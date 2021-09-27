#!/usr/bin/env python
# # coding=latin-1
from geometry_msgs.msg import Point, Quaternion, Pose
from actionlib_msgs.msg import *
import numpy as np
import math
from scipy.spatial.transform import Rotation as rot
from tf import transformations

import copy

class MyPoint(Point):
    def __init__(self, pos=(0.0, 0.0, 0.0)):
        if type(pos) == Point:
            self.asArray = np.array(pos.__reduce__()[2])
        else:
            self.asArray = np.array(pos)
        super(MyPoint, self).__init__(*(self.asArray))

        # 1 hinten anfuegen
        self.asArray = np.append(self.asArray, 0)

    def __add__(self, p2):
        p_out = MyPoint()
        p_out.x = self.x + p2.x
        p_out.y = self.y + p2.y
        p_out.z = self.z + p2.z

        return p_out

    def __sub__(self, p2):
        p_out = MyPoint()
        p_out.x = self.x - p2.x
        p_out.y = self.y - p2.y
        p_out.z = self.z - p2.z

        return p_out


class MyOrient(Quaternion):
    def __init__(self, quatern=(0.0, 0.0, 0.0, 1.0)):
        if type(quatern) == Quaternion:
            self._asArray = np.array(quatern.__reduce__()[2])
        else:
            self._asArray = np.array(quatern)
        super(MyOrient, self).__init__(*(self._asArray))

    def __add__(self, o2):
        return MyOrient(transformations.quaternion_multiply(self.asArray(), o2.asArray()))

    def __sub__(self, o2):
        q_inv = transformations.quaternion_conjugate(o2.asArray())
        return MyOrient(transformations.quaternion_multiply(self.asArray(), q_inv))

    def asArray(self):
        return np.array([self.x, self.y, self.z, self.w])


class MyPose(Pose):
    def __init__(self, pos=(0.0, 0.0, 0.0), quatern=(0.0, 0.0, 0.0, 1.0)):
        point = MyPoint(pos)
        orient = MyOrient(quatern)
        super(MyPose, self).__init__(point, orient)
        self.position = point
        self.orientation = orient

    def __add__(self, p2):
        p_out = MyPose()
        p_out.position = self.position + p2.position
        p_out.orientation = self.orientation + p2.orientation
        return p_out

    def __sub__(self, p2):
        p_out = MyPose()
        p_out.position = self.position - p2.position
        p_out.orientation = self.orientation - p2.orientation
        return p_out
    
    def rotateVector(self, vec=None, rot=None):
        if vec == None:
            vec = self.position.asArray
        if rot == None:
            rot = self.orientation.asArray
        rot_conj = transformations.quaternion_conjugate(rot)
        trans = transformations.quaternion_multiply(transformations.quaternion_multiply(rot_conj, vec), rot) [:3]
        return MyPoint(trans)


def rotateToXAxis(points, axis=(0,1), transpose = False):
    """Rotate list of points to from X-Axis to new axis 

    Args:
        points ([[[x1, y1]], [[x2,y2]],...]): list of points to rotate
        axis (tuple, optional): new x-Axis. Defaults to (0,1).
        transpose (bool, optional): reverse action. Defaults to False.

    Returns:
        points: rotated
    """
    if points[0] is None: return None

    axis = axis/np.linalg.norm(axis)
    R = np.mat([[axis[0],axis[1]],
                [-axis[1],axis[0]]])
    if transpose: R = R.T
    contour_new = np.array(np.matmul(points,R))
    return contour_new

def getOrientDiffNoZ(goalOrient=MyOrient(), preGripOrient=MyOrient(), withoutAxisIdx=2):
    """Calculates OrientDiff and sets rotation around axis to 0

    Args:
        goalOrient (MyOrient): goalPose. Defaults to MyOrient().
        preGripOrient (MyOrient, optional): initial Pose. If default: result=goalOrient without Axis. Defaults to MyOrient().
        withoutAxisIdx (int, optional): Axis to disregard rotation. Defaults to 2.

    Returns:
        MyOrient(withoutAxis), MyOrient(onlyAxis): For default: Orientation without Z and only Z
    """

    orient_delta = MyOrient((goalOrient-preGripOrient).asArray())
    return getOrientationNoZ(orient_delta, withoutAxisIdx)

def getOrientationNoZ(orient_delta=MyPose, withoutAxisIdx=2):
    """Sets rotation around axis to 0

    Args:
        orient_delta (MyOrient): Defaults to MyPose.
        withoutAxisIdx (int, optional): Axis to disregard rotation. Defaults to 2.

    Returns:
        MyOrient(withoutAxis), MyOrient(onlyAxis): For default: Orientation without Z and only Z
    """
    rot_delta = rot.from_quat(orient_delta.asArray())
    rot_delta_euler = rot_delta.as_euler("xyz")

    # set Z to 0
    rot_noZ_euler = copy.copy(rot_delta_euler)
    rot_noZ_euler[withoutAxisIdx] = 0
    rot_noZ = rot.from_euler("xyz",rot_noZ_euler)

    rot_onlyZ_euler = [0,0,0]
    rot_onlyZ_euler[withoutAxisIdx] = rot_delta_euler[withoutAxisIdx]
    rot_onlyZ = rot.from_euler("xyz",rot_onlyZ_euler)
    
    return MyOrient(rot_noZ.as_quat()), MyOrient(rot_onlyZ.as_quat())


def getNearestOrientation(goalOrient=MyOrient(), preGripOrient=[MyOrient()]):
    """Gets smallest orientation difference between goal and preGripOrientations
    by disregarding rotation around Z-Axis

    Args:
        goalOrient (MyOrient): Goal Orientation. Defaults to MyOrient().
        preGripOrient (list[MyOrient]): possible preGripOrientations. Defaults to [MyOrient()].

    Returns:
       idxNearest: int, Idx of nearest orientation
       deltaOrientOut: MyOrient, Orientation difference without Z
       deltaZOut: MyOrient, Orientation difference in Z
    """
    smallestAngle = np.Inf
    for i, gripOrient in enumerate(preGripOrient):
        deltaOrient, deltaZ = getOrientDiffNoZ(goalOrient, gripOrient)
        if math.acos(deltaOrient.w) < smallestAngle:
            deltaOrientOut = deltaOrient
            deltaZOut = deltaZ
            idxNearest = i
    return idxNearest, deltaOrientOut, deltaZOut

if __name__ == '__main__':
    p1 = MyPoint((1, 2, 3))
    p2 = MyPoint((1, 2, 3))

    p3 = p1 + p2

    print(p3)

    pose1 = MyPose((1, 2, 3))
    pose2 = MyPose((1, 2, 3))

    pose3 = MyPose()

    pose3.position = pose1.position + pose2.position

    print(pose3)
    
    
    p1 = MyPose((0.656036424314, -0.0597577841713, -0.103558385398), (-0.909901224555, 0.41268467068,
                                                                        -0.023065127793, 0.0352011934197))
    a1 = [-0.198922703533319, 1.3937412735955756, 0.11749296106956011, -1.312658217933717, -0.1588243463469876,
            2.762937863667806, 0.815807519980951]
    
    # panda_goal = PandaGoals(p1, a1)
    # print(panda_goal)
