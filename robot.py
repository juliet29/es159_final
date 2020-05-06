import pybullet
import numpy as np
from helper import midpoint, boundBox, drawCont


class Robot:
    """
    Control the motion of the robot 
    """
    def __init__(self, p, urdfPath, initPos):
        """Initialize the robot model.
        Args:
            p: pybullet library object
            urdf_path: string of the path to the robot urdf file
            base_pos: the robot base position
            base_orientation: the robot base orientation as a quaternion
        """
        # use the pybullet library 
        self.p = p
        # load the robot 
        self.id = self.p.loadURDF(urdfPath)
        self.numJoints = self.p.getNumJoints(self.id)
        # set to desired starting position, with end effector down 
        self.p.resetBasePositionAndOrientation(self.id, posObj=initPos, ornObj=[0, 0, 0, 1])
        # start tracking the robots's position
        self.baseInfo = self.p.getLinkState(bodyUniqueId=self.id, linkIndex=0)
        # make a bounding box of the robot's workspace (CW, NE->NW)
        self.vertices = [0.5, -1.5, 0.5, -0.5, -0.5, -0.5, -0.5, -1.5]
        (self.rBB, self.rBBC) = boundBox(self.baseInfo[0], self.vertices)
        # # show the bounding box 
        self.initBB = drawCont(self.p, self.rBB, [0,0,0])


    def translate(self, vel):
        # delete initial bounding box
        # self.p.removeUserDebugItem(self.initBB)
        # perform the motion 
        self.p.resetBaseVelocity(objectUniqueId = self.id, linearVelocity=vel)
        # # update robot's location 
        self.baseInfo = self.p.getLinkState(bodyUniqueId=self.id, linkIndex=0)
        # # update the bounding box
        (self.rBB, self.rBBC) = boundBox(self.baseInfo[0], self.vertices)






