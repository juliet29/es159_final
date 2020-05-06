import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from datetime import datetime
from scipy.spatial import distance
import helper as h
from geoPaths import path_df
from robot import Robot


# connect and set up the environment
physicsClient =  p.connect(p.DIRECT) # debug mode
#physicsClient =  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)

# reset cam
p.resetDebugVisualizerCamera(cameraDistance=8, cameraYaw=-90, cameraPitch=-30, cameraTargetPosition=[0,0,0])

# create workspace where geometry will be printed 
geoSpaceShape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[2,2,0])
geoSpaceBody = p.createMultiBody(baseCollisionShapeIndex=geoSpaceShape, basePosition=[0,0,0])

# initialize the robot and draw a bounding box 
kuka = Robot(p,"kuka_iiwa/model.urdf", [0, 3, 0])
# starting base position of robot 
basePos = kuka.baseInfo[0]

# show the desired geometry
# print('printing the geom')
# h.drawContDF(p, path_df, [1,1,1])

# robot will only be working in one hemisphere
posPathDF = path_df.loc[path_df['y'] >= 0]
negPathDF = path_df.loc[path_df['y'] < 0]

# show the hemisphere where robot will be working 
h.drawContDF(p,posPathDF, [0,0.5,0] )

# only want to consider points at z = 0 now
zeroDF = posPathDF.loc[posPathDF['z'] == 0]
# find the distance between the robot and the furthest point in its hemisphere
furthestPt = h.findFurthestPoint(zeroDF, basePos)
# draw the line between the robot and the furthest point 
p.addUserDebugLine(basePos, furthestPt, lineColorRGB=[0,1,0], lineWidth=1, lifeTime=0)

# create an object bounding box (oBB) whith the furthestPt at the NE edge
o_vertices = [0,0, 0,1, 1,1,1,0]
(oBB, oBBC) = h.boundBox(furthestPt, o_vertices)
h.drawCont(p, oBB, [1,1,1])


# tolerance for aligning the bounding boxes
tol = 0.001




# initial distances between oBBC and rBBC (the center of the object bounding box, and the robot bounding box )
xDist = distance.euclidean(oBBC[0],kuka.rBBC[0]) 
yDist = distance.euclidean(oBBC[1],kuka.rBBC[1])
x_align = False
y_align = False
t_exec = False
while (1):
    # align the x 
    if xDist > tol and not x_align:
        kuka.translate([-0.3,0,0])
        xDist = distance.euclidean(oBBC[0],kuka.rBBC[0])
    elif xDist <= tol and not x_align:
        kuka.translate([0,0,0])
        x_align = True
    # then align the y 
    elif yDist > tol and x_align and not y_align:
        kuka.translate([0,-0.3,0])
        yDist = distance.euclidean(oBBC[1],kuka.rBBC[1])
    elif yDist < tol and x_align and not y_align:
        kuka.translate([0,0,0])
        # update the bounding box
        h.drawCont(p, kuka.rBB, [0.1,0.5,0])
        print('not doing traj')
        y_align = True
    elif x_align and y_align and  not t_exec:
        # find points that are within the robots bounding box 
        currPoints = h.pointFilter(zeroDF, kuka.rBB)
        # execute the configurations for these points
        if not currPoints.empty:
            kuka.execTraj(currPoints)
            t_exec = True
            print("traj executed!")
        else:
            print("no points in this bounding box" )
    elif x_align and y_align and t_exec:
        print(" done executiong")
        # identify the next closest points


  
        
        

        # eval IK for points within rBB
        # execute those trajectories 
        # identify the next closest points 
        # redraw oBB around them 
        # repeat 
    
    p.stepSimulation()



        
#  execute the simulation
# while (1):
#     p.stepSimulation()
    



