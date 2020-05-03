import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data

# connect 
#physicsClient =  p.connect(p.DIRECT) # debug mode
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

# not using th computer's clock, so need to call the stepStimulation() command directly
useRealTimeSimulation = 0
p.setRealTimeSimulation(useRealTimeSimulation)


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

# run simulation through all configurations, and then stop 
iter = 0
while iter < len(configs): 
  for i in range(len(configs)):
    config = configs[i]
    
    iter +=1
    # reset the robot to this configuration 
    for n in range(numJoints):
      p.resetJointState(kukaId, n, config[n])

    # show the trajectory the robot is taking
    firstPos = poses[0]
    currPos = poses[i]
    black = [0,0,0]
    debugDuration = 15
    p.addUserDebugLine(firstPos, currPos, black, 1, debugDuration)

    # show current workspace position 
    currPosStr = str(poses[i])
    pauseTime = 1/5
    p.addUserDebugText(currPosStr, [1,1,1], black, lifeTime = pauseTime)

    # step the simulation 
    p.stepSimulation()
    
    # rest between each step 
    time.sleep(pauseTime)


# disconnect  
p.disconnect()