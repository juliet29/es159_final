import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data
from geoPaths import PATH, path_df
from scipy.spatial import distance


def midpoint(p1, p2):
    "Return the midpoint of 2 points in 3D space at z = 0"
    return (np.mean([p1[0], p2[0]]), np.mean([p1[1], p2[1]]), 0)

# environment variables 
path = PATH
debugColor = [0,0,0]

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

# load the robot
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
# reset base position and index to be 0
p.resetBasePositionAndOrientation(kukaId, [0, 3, 0], [0, 0, 0, 1])

# define robot workspace in world coordinates 
baseInfo = p.getLinkState(bodyUniqueId=kukaId, linkIndex=0)
basePos = baseInfo[0]
# robot bounding box (rBB) - vertices of a square defined clockwise from the NE vertex (in robot frame) -- based on robot workspace determined by observing its movement 
xR = basePos[0]
yR = basePos[1]
rBB = [(xR+0.5, yR-1.5, 0), (xR+0.5, yR-0.5, 0), (xR-0.5, yR-0.5, 0), (xR-0.5, yR-1.5, 0)]
for pt in range(len(rBB) -1):
    p.addUserDebugLine(rBB[pt], rBB[pt + 1], lineColorRGB=debugColor, lineWidth=1, lifeTime=0)


# robot will only be working in one hemisphere
posPathDF = path_df.loc[path_df['y'] >= 0]
negPathDF = path_df.loc[path_df['y'] < 0]

# show the hemisphere 
for point in range(posPathDF.shape[0]-1):
    currPoint = tuple(posPathDF.iloc[point])
    nextPoint = tuple(posPathDF.iloc[point + 1])
    p.addUserDebugLine(currPoint, nextPoint, lineColorRGB=[0,0.5,0], lineWidth=1, lifeTime=0)

# only want to consider points at z = 0 now
zeroDF = posPathDF.loc[posPathDF['z'] == 0]

# find the distance between the robot and the furthest point in its hemisphere
furthestPt = zeroDF.iloc[0]
for i in range(zeroDF.shape[0]):
  furthest_dist = distance.euclidean(basePos, furthestPt)
  if distance.euclidean(basePos, list(zeroDF.iloc[i])) > furthest_dist:
    furthestPt = zeroDF.iloc[i]
p.addUserDebugLine(basePos, furthestPt, lineColorRGB=[0,1,0], lineWidth=0.9, lifeTime=0)

# create an object bounding box (oBB) whith the furthestPt at the NE edge
xO = furthestPt[0]
yO = furthestPt[1]
oBB = [(xO, yO, 0), (xO, yO+1, 0), (xO+1, yO+1, 0), (xO+1, yO, 0)]
for pt in range(len(oBB) -1):
    p.addUserDebugLine(oBB[pt], oBB[pt + 1], lineColorRGB=[1,1,1,], lineWidth=1, lifeTime=0)

# find the centers of oBB, rBB 
oBBC = midpoint(oBB[0], oBB[2])
rBBC = midpoint(rBB[0], rBB[2])
p.addUserDebugLine(basePos, oBBC, lineColorRGB=[0,1,1], lineWidth=0.9, lifeTime=0)
p.addUserDebugLine(basePos, rBBC, lineColorRGB=[1,1,0], lineWidth=0.9, lifeTime=0)

# tolerance 
tol = 0.001

# execute the simulation
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)
# while (1):
arr = False
# move robot in y until the oBB and rBB are aligned
y_dist = distance.euclidean(oBBC[0],rBBC[0]) 
while (1):
    if y_dist > tol and arr==False:
        p.resetBaseVelocity(objectUniqueId = kukaId, linearVelocity=[-0.3,0,0])
        # check where base is 
        baseInfo = p.getLinkState(bodyUniqueId=kukaId, linkIndex=0)
        basePos = baseInfo[0]
        #print(basePos)
        xR = basePos[0]
        yR = basePos[1]
        rBB = [(xR+0.5, yR-1.5, 0), (xR+0.5, yR-0.5, 0), (xR-0.5, yR-0.5, 0), (xR-0.5, yR-1.5, 0)]
        rBBC = midpoint(rBB[0], rBB[2])
        y_dist = distance.euclidean(oBBC[0],rBBC[0])
    else:
        arr = True
        p.resetBaseVelocity(objectUniqueId = kukaId, linearVelocity=[0,0,0])
        print("here!")

        


        

    p.stepSimulation()

# # show the desired geometry
# print('printing the geom')
# for point in range(path_df.shape[0]-1):
#     currPoint = tuple(path_df.iloc[point])
#     nextPoint = tuple(path_df.iloc[point + 1])
#     p.addUserDebugLine(currPoint, nextPoint, lineColorRGB=debugColor, lineWidth=0.9, lifeTime=0)

