import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
from geoPaths import PATH
# connect 

#physicsClient =  p.connect(p.DIRECT) # debug mode
physicsClient =  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
planeId = p.loadURDF("plane.urdf") 
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])

# reset base position and index to be 0,
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])

# double check that the the kuka was actually imported 
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
  exit()
# print(numJoints)
# print(numJoints2)

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

# gravity in the simulation 
p.setGravity(0, 0, -9.8)

# not using th computer's clock, so need to call the stepStimulation() command directly
useRealTimeSimulation = 0
p.setRealTimeSimulation(useRealTimeSimulation)

# set orientation of ee to point down
orn = p.getQuaternionFromEuler([0, -math.pi, 0])


poses = PATH
# # notes on range of movement from base position 
# # x:  [-0.7, 0.5] (forward, backwards) (negative values make robot go forward)!
# # y: [-0.7, 0.7] (left ,right)
# # z:  [0, 1] (down, up)

# show the object to print


# get the joint space configurations for each element of poses
configs = []
for pos in poses:
  jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos, orn, ll, ul, jr, rp)
  configs.append(jointPoses)

# reset cam
p.resetDebugVisualizerCamera( cameraDistance=3, cameraYaw=-50, cameraPitch=-35, cameraTargetPosition=[0,0,0])
  # need to step through time instead of having array 
# run simulation through all configurations, and then stop 
iter = 0
while 1:
  blue = [0,1,3]
  for h in range(len(poses)-1):
          nextPos = poses[h + 1]
          p.addUserDebugLine(poses[h], nextPos, blue, 1, lifeTime=0)
  

  while iter < len(configs): 
    # change the velocity of the base, experimental for now 
    baseVel = p.getBaseVelocity(bodyUniqueId = kukaId)
    print("base1 = ", baseVel)

    #p.applyExternalForce(objectUniqueId = kukaId, linkIndex= -1, forceObj=[0,2,0], posObj=[0,0.5,0], flags=p.WORLD_FRAME)
    #p.resetBaseVelocity(objectUniqueId = kukaId, linearVelocity=[0,2,0])

    baseVel2 = p.getBaseVelocity(bodyUniqueId = kukaId)
    print("base2 = ", baseVel2)
    for i in range(len(configs)):
      config = configs[i]
      
      iter +=1
      # reset the robot to this configuration 
      for n in range(numJoints):
        # overrides all other physical simulations 
        #p.resetJointState(kukaId, n, config[n])
        #control joints to a state
        p.setJointMotorControl2(bodyIndex = kukaId, jointIndex=n, controlMode = p.POSITION_CONTROL, targetPosition=config[n])

      # show the trajectory the robot is taking
      
      red = [1,0,0]
      debugDuration = 15
      firstPos = poses[0]
      currPos = poses[i]
      # print current end effector pos
      ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
      p.addUserDebugLine(currPos, ls[4], red, 1, lifeTime=0)

      pauseTime = 1/300
      # show current workspace position 
      # currPosStr = str(poses[i])
      # p.addUserDebugText(currPosStr, [0,0,2], black, lifeTime = 0, replaceItemUniqueId=1)

      # step the simulation 
      p.stepSimulation()
      
      # rest between each step 
      time.sleep(pauseTime)
  # sleep when done then disconnect
  time.sleep(5)

  # disconnect  
  p.disconnect()