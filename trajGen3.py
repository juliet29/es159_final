import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
from geoPaths import PATH, path_df
from scipy.spatial import distance


# environment variables 
path = PATH
debugColor = [0,0,0]

# connect and set up the environment
#physicsClient =  p.connect(p.DIRECT) # debug mode
physicsClient =  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
planeId = p.loadURDF("plane.urdf")

# reset cam
p.resetDebugVisualizerCamera( cameraDistance=8, cameraYaw=90, cameraPitch=-30, cameraTargetPosition=[0,0,0])

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
# robot bounding box (rBB) - vertices of a square defined clockwise from the NE vertex (in robot frame)
x = basePos[0]
y = basePos[1]
rBB = [(x+0.5, y-1.5, 0), (x+0.5, y-0.5, 0), (x-0.5, y-0.5, 0), (x-0.5, y-1.5, 0)]
for pt in range(len(rBB) -1):
    p.addUserDebugLine(rBB[pt], rBB[pt + 1], lineColorRGB=debugColor, lineWidth=1, lifeTime=0)

# robot will only be working in one hemisphere
posPathDF = path_df.loc[path_df['y'] >= 0]
negPathDF = path_df.loc[path_df['y'] < 0]

# show the hemisphere 
for point in range(posPathDF.shape[0]-1):
    currPoint = tuple(posPathDF.iloc[point])
    nextPoint = tuple(posPathDF.iloc[point + 1])
    p.addUserDebugLine(currPoint, nextPoint, lineColorRGB=[0,1,0], lineWidth=0.9, lifeTime=0)

# only want to consider points at z = 0 now
zeroDF = posPathDF.loc[posPathDF['z'] == 0]

# find the distance between the robot and the furthest point in its hemisphere
furthestPt = zeroDF.iloc[0]
for i in range(zeroDF.shape[0]):
  furthest_dist = distance.euclidean(basePos, furthestPt)
  if distance.euclidean(basePos, list(zeroDF.iloc[i])) > furthest_dist:
    furthestPt = zeroDF.iloc[i]
print(furthestPt)
p.addUserDebugLine(basePos, furthestPt, lineColorRGB=[0,1,0], lineWidth=0.9, lifeTime=0)



# show the desired geometry
print('printing the geom')
for point in range(path_df.shape[0]-1):
    currPoint = tuple(path_df.iloc[point])
    nextPoint = tuple(path_df.iloc[point + 1])
    p.addUserDebugLine(currPoint, nextPoint, lineColorRGB=debugColor, lineWidth=0.9, lifeTime=0)

# execute the simulation
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)
while (1):
    p.stepSimulation()


