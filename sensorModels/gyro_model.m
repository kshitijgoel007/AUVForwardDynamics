function [WMeas, gyro_bias] = gyro_model(wb, gyro_bias, tinc)
% GYRO_MODEL generates measured angular velocity in [rad/s]
%  wb : Angular velocity of body expressed in body-frame [rad/s]
%  tinc : time step, [IMPORTANT : Depends upon sampling rate in case of actual IMU]
%  gyro_bias : previous value of bias

global gyroscope_bias_instability;
global gyroscope_ARW;
global gyro_corr_time;
global IMU_to_body;
global gyroscope_noise_density;

% % convert wb from [rad/s] to [deg/s]
% wb = wb*180/3.14;

% transform wb to IMU frame.
wb = IMU_to_body'*wb;

% Gyroscope random walk signal %
% gyroscope_sig_beta = gyroscope_random_walk*sqrt(tinc);
% gyroscope_bias += gyroscope_sig_beta*randn(3,1);

% Gyroscope bias instability, gauss markov process %
sigma_GM = sqrt(tinc/gyro_corr_time)*gyroscope_bias_instability;
gyro_bias = (1 - tinc/gyro_corr_time)*gyro_bias + sigma_GM*randn(3,1);

% Gyroscope white noise signal %
% gyroscope_white_noise = gyroscope_ARW*(1/sqrt(tinc))*randn(3,1);     %  White noise (deg/s)
gyroscope_noise_density_d = gyroscope_noise_density*(1/sqrt(tinc));
gyroscope_white_noise = gyroscope_noise_density_d*randn(3,1);

WMeas = wb + gyro_bias + gyroscope_white_noise;

end
