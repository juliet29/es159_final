## loads 2 robots and directly controls their position 

# activate virtual environment: source env159/bin/activate 

# docs: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.la294ocbo43o
# example: http://alexanderfabisch.github.io/pybullet.html 

import pybullet as p
import time
import pybullet_data

# set up the view 
physicsClient =  p.connect(p.GUI)  # turn on simulation 
#physicsClient =  p.connect(p.DIRECT)  # just print to command line 

# ability to get files from the intenet that are not downloaded locally :) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

# load a plane from py_bullet data 
planeId = p.loadURDF("plane.urdf")
hi = p.loadURDF("iiwa7.urdf")
# set starting locations for the two robots 
cubeStartPos = [0,0,0]
cubeStartPos2 = [2,2,0]

# load in a robot, also from pybullet data, use a fixed base or not 
robot = p.loadURDF("r2d2.urdf", cubeStartPos, useFixedBase=0)

# load in a second robot 
robot2 = p.loadURDF("r2d2.urdf", cubeStartPos2, useFixedBase=0)

# get orientation and position 
position, orientation = p.getBasePositionAndOrientation(robot)

# get number of joints 
joints = p.getNumJoints(robot)
print("joints: ", joints)

# understand more about about one joint 
joint_index = 1
joint_info = p.getJointInfo(robot, joint_index)
name, joint_type, lower_limit, upper_limit = joint_info[1], joint_info[2], joint_info[8], joint_info[9]
print("info about a joint: ", name, joint_type, lower_limit, upper_limit)

# get position of each joint
joint_positions = [j[0] for j in p.getJointStates(robot, range(6))]
print("all joint positions: ", joint_positions)

# get current positions of a given link 
world_position, world_orientation = p.getLinkState(robot, 2)[:2] # (vec3?)
print("world_position: ", world_position)

# now lets make something happen 

# add very heavy gravity in z direction lol 
p.setGravity(0, 0, -100) 

# step through time very slowly 
#p.setTimeStep(10e-3)  

# don't use the computer time  --> what happens if no? 
#p.setRealTimeSimulation(0)  

# make the joints of the robot go to a certain position 
p.setJointMotorControlArray(
    robot, range(12), p.POSITION_CONTROL,
    targetPositions= [10] * 12)

p.setJointMotorControlArray(
    robot2, range(12), p.POSITION_CONTROL,
    targetPositions=[5] * 12)

# setp through the simulation 
for _ in range(10000):
    p.stepSimulation()
    # slow things down by sleeping in between each execution 
    time.sleep(1./240.)

p.disconnect()





# cubeStartPos = [0,0,1]
# cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
# for i in range (10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
# p.disconnect()


