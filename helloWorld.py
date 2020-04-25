# "activate virtual environment: source env159/bin/activate "
import pybullet as p
import time
import pybullet_data

# set up the simulation
physicsClient =  p.connect(p.GUI)  

# ability to get files from the intenet that are not downloaded locally :) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

# turn on gravity in the z direction 
p.setGravity(0,0,-10)

# load a plane from py_bullet data 
planeId = p.loadURDF("plane.urdf")


cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()


