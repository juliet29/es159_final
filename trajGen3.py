import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data
from scipy.spatial import distance
from geoPaths import path_df
from robot import Robot
from helper import midpoint, boundBox, drawCont, drawContDF


# connect and set up the environment
#physicsClient =  p.connect(p.DIRECT) # debug mode
physicsClient =  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)

# reset cam
p.resetDebugVisualizerCamera( cameraDistance=8, cameraYaw=-90, cameraPitch=-30, cameraTargetPosition=[0,0,0])

# create workspace where geometry will be printed 
geoSpaceShape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[2,2,0])
geoSpaceBody = p.createMultiBody(baseCollisionShapeIndex=geoSpaceShape, basePosition=[0,0,0])

# initialize the robot and draw a bounding box 
kuka = Robot(p,"kuka_iiwa/model.urdf", [0, 3, 0])
# starting base position of robot 
basePos = kuka.baseInfo[0]

# show the desired geometry
# print('printing the geom')
# drawContDF(p, path_df, [1,1,1])

# robot will only be working in one hemisphere
posPathDF = path_df.loc[path_df['y'] >= 0]
negPathDF = path_df.loc[path_df['y'] < 0]

# show the hemisphere where robot will be working 
# drawContDF(p,posPathDF, [0,0.5,0] )

# only want to consider points at z = 0 now
zeroDF = posPathDF.loc[posPathDF['z'] == 0]
# find the distance between the robot and the furthest point in its hemisphere
furthestPt = zeroDF.iloc[0]
for i in range(zeroDF.shape[0]):
  furthest_dist = distance.euclidean(basePos, furthestPt)
  if distance.euclidean(basePos, list(zeroDF.iloc[i])) > furthest_dist:
    furthestPt = zeroDF.iloc[i]
# draw the line between the robot and the furthest point 
p.addUserDebugLine(basePos, furthestPt, lineColorRGB=[0,1,0], lineWidth=1, lifeTime=0)

# create an object bounding box (oBB) whith the furthestPt at the NE edge
o_vertices = [0,0, 0,1, 1,1,1,0]
(oBB, oBBC) = boundBox(furthestPt, o_vertices)
drawCont(p, oBB, [1,1,1])

# show centers of oBB, rBB 
p.addUserDebugLine(basePos, oBBC, lineColorRGB=[0,1,1], lineWidth=0.9, lifeTime=0)
p.addUserDebugLine(basePos, kuka.rBBC, lineColorRGB=[1,1,0], lineWidth=0.9, lifeTime=0)

# tolerance 
tol = 0.001

#  execute the simulation
# while (1):
#     p.stepSimulation()


# move robot in x until the oBB and rBB are aligned
xDist = distance.euclidean(oBBC[0],kuka.rBBC[0]) 
x_align = False
while (1):
    if xDist > tol and x_align == False:
        kuka.translate([-0.3,0,0])
        baseVel = p.getBaseVelocity(bodyUniqueId = kuka.id)
        xDist = distance.euclidean(oBBC[0],kuka.rBBC[0])
 
    else:
        x_align = True
        kuka.translate([0,0,0])
        # draw the new bounding box 
        drawCont(p, kuka.rBB, [0.1,0.5,0])
    p.stepSimulation()



        

    



