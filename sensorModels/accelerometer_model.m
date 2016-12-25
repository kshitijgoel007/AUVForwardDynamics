% Three axis Accelerometer model - [22-12-2016]
% https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model (some issue with
% noise density)
% An introduction to inertial navigation by Oliver J. Woodman
% A comparison between different error modelling of MEMS applied to GPS/INS
% integerated system

function [a_meas , accelerometer_bias ] = accelerometer_model(wb, wb_dot, Acc_true, accelerometer_bias, tinc)
% ACCELEROMETER_MODEL generates measured acceleration data [m/s2]
%  wb : Angular velocity of body expressed in body-frame [rad/s]
%  wb_dot : Angular acceleration of body expressed in body-frame [rad/s2]
%  Acc_true : True value of acceleration (non-gravitational part)
%  tinc : time step, [IMPORTANT : Depends upon sampling rate in case of actual IMU]
%  accelerometer_bias : previous value of bias

global accelerometer_location;
global r_cg;
global accelerometer_bias_instability;
global IMU_Accelerometer_SF_MA;
global accelerometer_VRW;
global accel_corr_time;

d = [(accelerometer_location(1) -  r_cg(1));
     (accelerometer_location(2) -  r_cg(2));
     (accelerometer_location(3) -  r_cg(3))];
 
Aimeas = Acc_true + cross(wb,cross(wb,d)) + cross(wb_dot,d);


% Tranform Aimeas to IMU frame.


% Accelerometer random walk signal %
% accelerometer_sig_beta = acceleration_random_walk*sqrt(tinc);
% accelerometer_beta += acceleration_sig_beta*randn(3,1);


% Accelerometer bias instability, gauss markov process %
sigma_GM = sqrt(tinc/accel_corr_time)*accelerometer_bias_instability;
accelerometer_bias = (1 - tinc/accel_corr_time)*accelerometer_bias + sigma_GM*randn(3,1);

% Accelerometer white noise signal %
accelerometer_white_noise = accelerometer_VRW*(1/sqrt(tinc))*randn(3,1);     %  White noise (m/s2)

a_meas = (eye(3,3)+ IMU_Accelerometer_SF_MA)*Aimeas + accelerometer_bias + accelerometer_white_noise;
a_meas = a_meas';
accelerometer_bias = accelerometer_bias';

end