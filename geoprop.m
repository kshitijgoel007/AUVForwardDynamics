global W B L Ix Iy Iz Ixy Iyz Ixz Iyx Izy Izx xg x_b yg y_b zg z_b rho m g maxrudd max_rudd_rate del_o Xold;
W = 53400;
B = 53400;
L = 5.3;
Ix = 2038;
Iy = 13587;
Iz = 13587;
Ixy = -13.58;
Iyz = -13.58;
Ixz = -13.58;
Iyx = Ixy;
Izy = Iyz;
Izx = Ixz;
xg = 0;
yg = 0;
zg = 0.061;
x_b = 0;
y_b = 0;
z_b = 0;
rho = 1000;
m = 5454.54;
g = 9.81;

%SOME ADDN PARAMETERS FOR CONTROL SURFACE DEFLECTION

maxrudd=35*pi/180;
max_rudd_rate=2.5*pi/180;
del_o=0;
dt=.1;