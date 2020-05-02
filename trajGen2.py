import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import cmath

# clid = p.connect(p.SHARED_MEMORY)
# if (clid < 0):
#p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

# connect 
#physicsClient =  p.connect(p.DIRECT)
physicsClient =  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
planeId = p.loadURDF("plane.urdf") 
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])

# rese base position and index to be 0,
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])

# double check that the the kuka was actually imported 
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
  exit()

# will eventually want to be able to move the joints -- kuka_iiwa/model_free_base.urdf

# set limits for the null space (where the robot cannot go --> how is this determined 
#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

# reset the joint states of the robot for each joint
for i in range(numJoints):
  p.resetJointState(kukaId, i, rp[i])

# no gravity simulation for now 
p.setGravity(0, 0, 0)

# starting time is 0
t = 0.01

# set up previous positions, and the prior previous position
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
# does not have a prev pose, set this to false
hasPrevPose = 1

# use the null space 
useNullSpace = 0
# orientation points down if 1, not defined if 0
useOrientation = 1

#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.

# currently we are using dynamic control
useSimulation = 1 
# not using th computer's clock, so need to call the stepStimulation() command directly
useRealTimeSimulation = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15

# ikSolver, not sure this will do
ikSolver = 0

# set orientation of ee to point down
orn = p.getQuaternionFromEuler([0, -math.pi, 0])

# create a list of not time dependent positions to go through 
poses = []
for i in range(1,10):
  poses.append([-0.4, 0.1*i, 0. + 0.2])

# get the joint space configurations for each element of poses
configs = []
for pos in poses:
  jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos, orn, ll, ul, jr, rp)
  configs.append(jointPoses)

# run simulation through our configurations 
iter = 0
while iter < len(configs): # reset this later 
  for i in range(len(configs)):
    config = configs[i]
    
    iter +=1
    # reset the robot to this configuration 
    print(config)
    for i in range(numJoints):
            p.resetJointState(kukaId, i, config[i])


    ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
    # draw line from previous workspace position to current ws position
    
    # will want to change to last pos implementation 
    currPos = poses[i]
    nextPos = poses[i + 1]
    p.addUserDebugLine(currPos, nextPos, [0, 0, 0.3], 1, trailDuration)
    #p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)

    # step the simulation 
    p.stepSimulation()
    
    time.sleep(1/10)







  
p.disconnect()