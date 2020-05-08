# ENG-SCI159: Introduction to Robotics Final Project: Robotic 3D Printing for Construction 

## Important Files 
* trajGen2.py - PyBullet simulation of Kuka Iiwa robot printing a geometry in it's own workspace. 
* trajGen3.py - PyBullet simulation of Kuka Iiwa roprinting a geometry that is larger than its workspace 
* extractPaths_orig.ipynb - Python notebook that shows visualization of the paths being extracted from the geometry, and motion planning of the robot 
* robot.py - robot class that controls the robot's motion 
* helper.py - helper functions
* geoPaths.py - file that create paths for the robot to print from a given geometry 


## Abstract 
Recent innovations in construction feature the use of robotic 3D printers to construct buildings with greater speed, efficiency, and affordability than conventional methods.  However, these robots are often installed on a large gantry system, limiting the scale of the project being built to what the gantry can reach. Additionally, the sheer scale of the gantry systems make them difficult to move easily and limit their effectiveness. To be effective, particularly in parts of the world where  there are critical housing shortages, a robot for construction should be lightweight, robust, and mobile, everything a gantry system is not. One proposal [1]   suggests using a system of two mobile robots instead of a gantry. The authors show that with such a system, the workspace available to the total system is greater than each robot's individual workspace. In this work, I replicate aspects of this paper through simulation in Python Bullet. My emphasis is on writing a path planning algorithm that, upon execution, shows robots constructing a geometric object that is greater than their workspace. 

[1]	X. Zhang et al., ‘Large-scale 3D printing by a team of mobile robots’, Automation in Construction, vol. 95, pp. 98–106, Nov. 2018, doi: 10.1016/j.autcon.2018.08.004.
