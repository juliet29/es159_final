import pybullet
import numpy as np

def midpoint(p1, p2):
    "Return the midpoint of 2 points in 3D space at z = 0"
    return (np.mean([p1[0], p2[0]]), np.mean([p1[1], p2[1]]), 0)


class Robot:
    """
    Control the motion of the robot 
    """
    def __init__(self, p, urdf_path, base_pos=[0, 0, 0],
                 base_orientaion=[0, 0, 0, 1]):
        """Initialize the robot model.
        Args:
            p: pybullet library object
            urdf_path: string of the path to the robot urdf file
            base_pos: the robot base position
            base_orientation: the robot base orientation as a quaternion
        """
        self.p = p
        self.id = self.p.loadURDF(urdf_path, base_pos, base_orientaion,
                    useFixedBase=1,
                    flags=pybullet.URDF_MERGE_FIXED_LINKS |
                          pybullet.URDF_USE_SELF_COLLISION)
        self.num_joints = self.p.getNumJoints(self.id)

    def translate(vel):
        # perform the motion 
        p.resetBaseVelocity(objectUniqueId = kukaId, linearVelocity=vel)
        # update robot's location 
        baseInfo = p.getLinkState(bodyUniqueId=kukaId, linkIndex=0)
        basePos = baseInfo[0]
        # update rBB position 
        xR = basePos[0]
        yR = basePos[1]
        rBB = [(xR+0.5, yR-1.5, 0), (xR+0.5, yR-0.5, 0), (xR-0.5, yR-0.5, 0), (xR-0.5, yR-1.5, 0)]
        rBBC = midpoint(rBB[0], rBB[2])

        return (basePos, rBBC)





