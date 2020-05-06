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
drawContDF(p,posPathDF, [0,0.5,0] )


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

# # execute the simulation
# p.setGravity(0, 0, -10)
# p.setRealTimeSimulation(0)
while (1):
    p.stepSimulation()
    #time.sleep(5)
# arr = False
# # move robot in y until the oBB and rBB are aligned
# y_dist = distance.euclidean(oBBC[0],rBBC[0]) 
# while (1):
#     if y_dist > tol and arr==False:
#         p.resetBaseVelocity(objectUniqueId = kukaId, linearVelocity=[-0.3,0,0])
#         # check where base is 
#         # baseInfo = p.getLinkState(bodyUniqueId=kukaId, linkIndex=0)
#         # basePos = baseInfo[0]
#         # xR = basePos[0]
#         # yR = basePos[1]
#         # rBB = [(xR+0.5, yR-1.5, 0), (xR+0.5, yR-0.5, 0), (xR-0.5, yR-0.5, 0), (xR-0.5, yR-1.5, 0)]
#         # rBBC = midpoint(rBB[0], rBB[2])
#         # print(rBBC)
#         y_dist = distance.euclidean(oBBC[0],rBBC[0])
#     else:
#         arr = True
#         p.resetBaseVelocity(objectUniqueId = kukaId, linearVelocity=[0,0,0])
#         print("here!")

    # need to write functions 
    #      


        

    



