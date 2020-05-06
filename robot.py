import pybullet
import numpy as np
from helper import midpoint, boundBox, drawCont


class Robot:
    """
    Control the motion of the robot 
    """
    def __init__(self, p, urdf_path, base_pos,
                 base_orientaion=[0, 0, 0, 1]):
        """Initialize the robot model.
        Args:
            p: pybullet library object
            urdf_path: string of the path to the robot urdf file
            base_pos: the robot base position
            base_orientation: the robot base orientation as a quaternion
        """
        # for pyBullet purposes 
        self.p = p
        # load the robot 
        self.id = self.p.loadURDF(urdf_path, base_pos, base_orientaion)
        self.num_joints = self.p.getNumJoints(self.id)
        # start tracking the robots's position
        self.baseInfo = self.p.getLinkState(bodyUniqueId=self.id, linkIndex=0)
        # make a bounding box of the robot's workspace (CW, NE->NW)
        self.vertices = [0.5, -1.5, 0.5, -0.5, -0.5, -0.5, -0.5, -1.5]
        (self.rBB, self.rBBC) = boundBox(self.baseInfo[0], self.vertices)
        # # show the bounding box 
        drawCont(self.p, self.rBB, [0,0,0])


    def translate(vel):
        # perform the motion 
        self.p.resetBaseVelocity(objectUniqueId = self.id, linearVelocity=vel)
        # update robot's location 
        self.baseInfo = self.p.getLinkState(bodyUniqueId=self.id, linkIndex=0)
        # update the bounding box
        (self.rBB, self.rBBC) = boundBox(self.baseInfo[0], self.vertices)
        drawCont(self.p, self.rBB, [0,0,0])





