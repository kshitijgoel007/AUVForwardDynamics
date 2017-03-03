'''
Ref : http://docs.sympy.org/dev/modules/physics/vector/api/classes.html
'''
## Search for check, TODO for further work

from sympy.physics.mechanics import *
from sympy import symbols
from sympy import simplify
from sympy import Matrix
import numpy as np
from sympy.solvers import solve
from sympy.physics.vector import init_vprinting
from sympy.physics.vector import vlatex, vpprint, vprint
from sympy.physics.vector import kinematic_equations
# from sympy import subs
from sympy import init_printing

init_vprinting(use_latex='mathjax', pretty_print=True, use_unicode=True)
init_printing(use_latex='mathjax', pretty_print=True, use_unicode=True)

print 'Calculation of kinematic model for state estimation with states X, Y, Z, Roll, Pitch, Yaw, Vx, Vy, Vz \n'

'''
Reference frames
---------------

N : Newtonian frame
T : tanget frame or NED frame
B : Body frame
IMU : IMU frame
DVL : DVL frame
'''
N = ReferenceFrame('N')
T = ReferenceFrame('T')
B  = ReferenceFrame('B')
IMU  = ReferenceFrame('IMU')
DVL  = ReferenceFrame('DVL')

'''
Generalized coordinate and speeds
--------------

psi, theta, phi are euler angles of body frame B of AUV
x, y, z are location of bo, c.g of AUV.
'''

psi, theta, phi, x, y, z = dynamicsymbols(' psi theta phi x y z')
psi_d, theta_d, phi_d, x_d, y_d, z_d = dynamicsymbols('psi theta phi x y z', 1)
p, q, r, u, v, w = dynamicsymbols('p q r u v w') # Body rates and velocities

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
t = symbols('t')

## location of IMU and DVL from b0
l_IMU_x, l_IMU_y, l_IMU_z = symbols('l_IMU_x l_IMU_y l_IMU_z')
l_DVL_x, l_DVL_y, l_DVL_z = symbols('l_DVL_x l_DVL_y l_DVL_z')

'''
Orientation of Reference Frames
'''

B.orient(T, 'Body', [psi, theta, phi], '321')
IMU.orient(B, 'Body', [psi_IMU, theta_IMU, phi_IMU], '321')
DVL.orient(B, 'Body', [psi_DVL, theta_DVL, phi_DVL], '321')
# print vpprint(IMU.dcm(B))

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
Angular velocities
'''

# kinematic Differential equation
kde = kinematic_equations([p, q, r], [psi, theta, phi], 'body', '321')
# dq_dict = solve(kde, [p, q, r], set=True)
# B.set_ang_vel(T,.subs(dq_dict))
# print B.ang_vel_in(T)
DVL.set_ang_vel(B, 0*B.x)
IMU.set_ang_vel(B, 0*B.x)

'''
Setup linear velocities
'''


# origin is fixed
to.set_vel(T, 0)

# set velocities of mass centers
bo.set_vel(T, bo.pos_from(to).dt(T))
io.v2pt_theory(bo, T, IMU)
do.v2pt_theory(bo, T, DVL)
# print B.dcm(T)
print "\n ..... hello ...... \n"

print bo.pos_from(to).express(B).diff(t, B)
print vpprint(bo.pos_from(to).express(B).diff(t, T).to_matrix(B))

# print bo.pos_from(to)*B.dcm(T)
# print vprint(bo.pos_from(to).express(B))
# print B.partial_velocity(T, phi_d, psi_d, theta_d)

'''
setup acceleration
'''
bo.set_acc(T, bo.vel(T).dt(T))
io.a2pt_theory(bo, T, IMU)
do.a2pt_theory(bo, T, DVL)

# print io.partial_velocity(T, )
# print io.pos_from(to).express(T)
# print B.partial_velocity(T, x_d, y_d, z_d, phi_d, theta_d, psi_d)
# print express(IMU.ang_vel_in(T), T)
