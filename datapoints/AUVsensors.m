global r_cg d_IMU d_DVL;
global earth_rate R_i2t;
global gravity;

gravity = 9.81; % m/s2 [should be defined earlier than scripts mentioned below ]

IMUparams;
DVLparams;
PSensorparams;

earth_rate = [ 0 0 0]';
R_i2t = [1 0 0
         0 1 0
         0 0 1];
     
r_cg = [0; 0; 0] *1e-3; %[m]

d_IMU = [(accelerometer_location(1) -  r_cg(1));
         (accelerometer_location(2) -  r_cg(2));
         (accelerometer_location(3) -  r_cg(3))];
     
d_DVL = [(dvl_location(1) -  r_cg(1));
         (dvl_location(2) -  r_cg(2));
         (dvl_location(3) -  r_cg(3))];
