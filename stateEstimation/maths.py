'''
Ref : http://docs.sympy.org/dev/modules/physics/vector/api/classes.html
'''
## Search for check, TODO for further work

from sympy.physics.mechanics import *
from sympy import symbols
from sympy.physics.vector import init_vprinting
from sympy import init_printing

init_vprinting(use_latex='mathjax', pretty_print=True)
init_printing(use_latex='mathjax', pretty_print=True)

print 'Calculation of kinematic model for state estimation with states X, Y, Z, Roll, Pitch, Yaw, Vx, Vy, Vz \n'

'''
Reference frames
---------------

N : Newtonian frame
T : tanget frame or NED frame
B : Body frame
IMU : IMU frame
DVL : DVL frame

B1 and B2 are intermediate frames such that, B -> B2 -> B1 -> B
IMU1 and IMU2 are intermediate frames such that, B -> IMU2 -> IMU1 -> IMU
DVL1 and DVL2 are intermediate frames such that, B -> DVL2 -> DVL1 -> DVL
'''
N = ReferenceFrame('N')
T = ReferenceFrame('T')

B2 = ReferenceFrame('B2')
B1 = ReferenceFrame('B1')
B  = ReferenceFrame('B')

IMU1 = ReferenceFrame('IMU2')
IMU2 = ReferenceFrame('IMU1')
IMU  = ReferenceFrame('IMU')

DVL2 = ReferenceFrame('DVL2')
DVL1 = ReferenceFrame('DVL1')
DVL  = ReferenceFrame('DVL')

'''
Generalized coordinate and speeds
--------------

psi, theta, phi are euler angles of body frame B of AUV
x, y, z are location of bo, c.g of AUV.
'''

psi, theta, phi, x, y, z = dynamicsymbols(' psi theta phi x y z')
psi_d, theta_d, phi_d, x_d, y_d, z_d = dynamicsymbols('psi theta phi x y z', 1)
u1, u2, u3, u4, u5, u6 = dynamicsymbols('u1:7')

'''
constants and Parameters
---------------

psi_IMU, theta_IMU, phi_IMU  : describes Orientation of IMU w.r.t BF
psi_DVL, theta_DVL, phi_DVL : describes Orientation of DVL w.r.t BF
l_IMU_x, l_IMU_y, l_IMU_z : describes position of IMU w.r.t bo
l_DVL_x, l_DVL_y, l_DVL_z : describes postion of IMU w.r.t bo

'''

## constant euler angles
psi_IMU, theta_IMU, phi_IMU = symbols(' psi_IMU theta_IMU phi_IMU')
psi_DVL, theta_DVL, phi_DVL = symbols(' psi_DVL theta_DVL phi_DVL')

## location of IMU and DVL from b0
l_IMU_x, l_IMU_y, l_IMU_z = symbols('l_IMU_x l_IMU_y l_IMU_z')
l_DVL_x, l_DVL_y, l_DVL_z = symbols('l_DVL_x l_DVL_y l_DVL_z')

'''
Orientation of Reference Frames
'''

# T -> B2 -> B1 -> B
B2 = T.orientnew('B2', 'Axis', [psi, T.z])
B1 = B2.orientnew('B1', 'Axis', [theta, B2.y])
B = B1.orientnew('B', 'Axis', [phi, B1.x]) # Body frame

# print B.dcm(T)

# B.orient(T, 'Body', [psi, theta, phi], '321')

# B -> IMU2 -> IMU1 -> IMU
IMU2 = B.orientnew('IMU2', 'Axis', [psi_IMU, B.z])
IMU1 = IMU2.orientnew('IMU1', 'Axis', [theta_IMU, IMU2.y])
IMU  = IMU1.orientnew('IMU', 'Axis', [phi_IMU, IMU1.x]) # IMU frame

# B -> DVL2 -> DVL1 -> DVL
DVL2 = B.orientnew('DVL2', 'Axis', [psi_DVL, B.z])
DVL1 = DVL2.orientnew('DVL1', 'Axis', [theta_DVL, DVL2.y])
DVL  = DVL1.orientnew('DVL', 'Axis', [phi_DVL, DVL1.x]) # DVL frame

'''
Position vectors
to : tangent frame origin
bo : body frame origin (body c.g)
io : IMU c.g (IMU frame origin)
do : DVL c.g (DVL frame origin)
'''

to = Point('to')
bo = Point('bo')
io = Point('io')
do = Point('do')

# Tangent frame origin to body frame origin
bo.set_pos(to, x*T.x + y*T.y + z*T.z)

# body frame origin to imu frame origin
io.set_pos(bo, l_IMU_x*B.x + l_IMU_y*B.y + l_IMU_z*B.z)

# body frame origin to dvl frame origin
do.set_pos(bo, l_DVL_x*B.x + l_DVL_y*B.y + l_DVL_z*B.z)

'''
Kinematic diff. Equations
'''
kinematical = [psi_d - u1,
               theta_d - u2,
               phi_d - u3,
               x_d - u4,
               y_d - u5,
               z_d - u6]
'''
Angular velocities
'''
B2.set_ang_vel(T, psi*T.z)
B1.set_ang_vel(B2, theta*B2.y)
B.set_ang_vel(B1, phi*B1.x)

DVL2.set_ang_vel(B, 0*B.z)
DVL1.set_ang_vel(DVL2, 0*DVL2.y)
DVL.set_ang_vel(DVL1, 0*DVL1.x)

IMU2.set_ang_vel(B, 0*B.z)
IMU1.set_ang_vel(IMU2, 0*IMU2.y)
IMU.set_ang_vel(IMU1, 0*IMU1.x)

'''
Setup linear velocities
'''

# origin is fixed
to.set_vel(T, 0)

# set velocities of mass centers
bo.set_vel(T, bo.pos_from(to).dt(T))
io.v2pt_theory(bo, T, IMU)
do.v2pt_theory(bo, T, DVL)
# print io.vel(T)
# print B.ang_vel_in(T)
# print express(IMU.ang_vel_in(T), T)

'''
Constants
'''
## Define constants and variables TODO : include g
constants = [
             psi_IMU,
             theta_IMU,
             phi_IMU,
             psi_DVL,
             theta_DVL,
             phi_DVL,
             l_IMU_x,
             l_IMU_y,
             l_IMU_z,
             l_DVL_x,
             l_DVL_y,
             l_DVL_z]
# coordinates = [psi, theta, phi, x, y, z]
# speeds = [psi_d, theta_d, phi_d, x_d, y_d, z_d]

# print B0.pos_from(T0)
# print B0.pos_from(T0).express(B)

# print IMU.dcm(B)
# print T.dcm(B)
# print T.dcm(B)
