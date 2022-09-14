#!/usr/bin/python3

from math import sin, cos, pi
import numpy as np

def getJacobianPlatform():
    J = np.zeros((6, 2))
    J[0,0]=1
    J[5,1]=1
    return J

def getRotationMatrix(theta):
    R = np.zeros((3, 3))
    R[0,0]=cos(theta)
    R[0,1]=-sin(theta)
    R[1,0]=sin(theta)
    R[1,1]=cos(theta)
    R[2,2]=1
    return R

def getRotationMatrixS3(theta):
    G = np.zeros((6, 6))
    G[0:3,0:3]=getRotationMatrix(theta)
    G[3:6,3:6]=getRotationMatrix(theta)
    return G

def getJacobianPlatformToMap(theta):
    # J = np.zeros((6, 2))
    # J[0,0]=cos(theta)
    # J[1,0]=sin(theta)
    # J[5,1]=1
    J=getRotationMatrixS3(theta)@getJacobianPlatform()
    return J

if __name__ == "__main__":
    theta = pi/2
    print(getJacobianPlatform(theta))