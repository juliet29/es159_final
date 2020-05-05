import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
from geoPaths import PATH


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
for pt in range(len(rBB) -1 ):
    p.addUserDebugLine(rBB[pt], rBB[pt + 1], lineColorRGB=debugColor, lineWidth=1, lifeTime=0)


# print("base_world_pos: ", baseLoc[0])

# print the desired geometry
print('printing the geom')
for point in range(len(path)-1):
    currPoint = path[point]
    nextPoint = path[point + 1]
    p.addUserDebugLine(currPoint, nextPoint, lineColorRGB=debugColor, lineWidth=0.9, lifeTime=0)

# execute the simulation
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)
while (1):
    p.stepSimulation()


