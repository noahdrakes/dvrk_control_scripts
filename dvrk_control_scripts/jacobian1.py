# example of Python script using cisstRobotPython

import cisstRobotPython
import numpy as np

# create a manipulator
r = cisstRobotPython.robManipulator()

# load DH parameters from a .rob file, result is 0 if loaded properly 
r.LoadRobot('/home/ndrakes1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob')

# check the length of the kinematic chain
#print('length of kinematic chain: %d' % len(r.links))


#  Jacobian 

jp = np.zeros(6)
J_body = np.zeros(shape=(6,6), dtype=np.double)
r.JacobianBody(jp, J_body)
print("Jacobian: ")
print(J_body)

# position when all joints are at 0
#jp = np.zeros(6)
#cp = np.zeros(shape = (4, 4)) # 4x4 matrix
#cp = r.ForwardKinematics(jp)
#print('joint position:')
#print(jp)
#print('cartesian position:')
#print(cp)

# for a PSM, joint 3 is along instrument shaft
#print('cartesian position for joint position [0,0,0.08,0,0,0]')
#jp[2] = 0.08 # 8cm
#cp = r.ForwardKinematics(jp)
#print('joint position:')
#print(jp)
#print('cartesian position:')
#print(cp)
