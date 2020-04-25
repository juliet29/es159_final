import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import pybullet as p
import time
import pybullet_data


fig = plt.figure()
ax = fig.gca(projection='3d')

# Make data.
X = np.arange(-5, 5, 0.25)
Y = np.arange(-5, 5, 0.25)
X, Y = np.meshgrid(X, Y)
R = np.sqrt(X**2 + Y**2)
Z = np.sin(R)

# # Plot the surface.
# surf = ax.plot_surface(X, Y, Z)

# plt.show()

# convert to an obj file 


#### for pybullet now ##########


# set up the view 
physicsClient =  p.connect(p.GUI)  # turn on simulation 
#physicsClient =  p.connect(p.DIRECT)  # just print to command line 

# ability to get files from the intenet that are not downloaded locally :) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
#planeId = p.loadURDF('plane.urdf')

# make a visual shape id 
visualShapeId = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName='plant.obj',
    rgbaColor=None,
    meshScale=[10e-4, 10e-4, 10e-4])

# create a collision shape id 
collisionShapeId = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName='plant.obj',
    meshScale=[0.1, 0.1, 0.1])

multiBodyId = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=collisionShapeId, 
    baseVisualShapeIndex=visualShapeId,
    basePosition=[0, 0, 1],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

for _ in range(10000):
    # slow things down by sleeping in between each execution
    p.stepSimulation() 
    time.sleep(1./240.)