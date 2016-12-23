% Three axis Accelerometer model - [22-12-2016]
% https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
% An introduction to inertial navigation by Oliver J. Woodman

function a_meas = accelerometer_model(wb, wb_dot, Acc_true, tinc)
% ACCELEROMETER_MODEL generates measured acceleration data [m/s2]
%  wb : Angular velocity of body expressed in body-frame [rad/s]
%  wb_dot : Angular acceleration of body expressed in body-frame [rad/s2]
%  tinc : time step, [IMPORTANT : Depends upon sampling rate in case of actual IMU]
%  bias : previous value of bias

global accelerometer_location;
global r_cg;
global accelerometer_bias_instability;
global accelerometer_noise_density;
global IMU_Accelerometer_SF_MA;

d = [(accelerometer_location(1) -  r_cg(1));
     (accelerometer_location(2) -  r_cg(2));
     (accelerometer_location(3) -  r_cg(3))];
 
Aimeas = Acc_true + cross(wb,cross(wb,d)) + cross(wb_dot,d);

% Accelerometer bias signal %
% accelerometer_sig_beta = acceleration_random_walk*sqrt(tinc);
% accelerometer_beta += acceleration_sig_beta*randn(3,1);
accelerometer_beta = accelerometer_bias_instability*randn(3,1);

% Accelerometer white noise signal %
accelerometer_noise_density_d = accelerometer_noise_density*(1/sqrt(tinc));
accelerometer_white_noise = accelerometer_noise_density_d*randn(3,1);     %  White noise (m/s2)

a_meas = (eye(3,3)+ IMU_Accelerometer_SF_MA)*Aimeas + accelerometer_white_noise + accelerometer_beta;

end