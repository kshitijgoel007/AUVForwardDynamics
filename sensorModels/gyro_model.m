function WMeas = gyro_model(wb, tinc)
% GYRO_MODEL generates measured angular velocity in [deg/s]
%  wb : Angular velocity of body expressed in body-frame [rad/s]
%  tinc : time step, [IMPORTANT : Depends upon sampling rate in case of actual IMU]

global gyroscope_bias_instability;
global gyroscope_noise_density;

% convert wb from [rad/s] to [deg/s]
wb = wb*180*(1/3.14);

% Gyroscope bias signal %
% gyroscope_sig_beta = gyroscope_random_walk*sqrt(tinc);
% gyroscope_bias += gyroscope_sig_beta*randn(3,1);
gyroscope_bias = gyroscope_bias_instability*randn(3,1);

% Gyroscope white noise signal %
gyroscope_noise_density_d = gyroscope_noise_density*(1/sqrt(tinc));
gyroscope_white_noise = gyroscope_noise_density_d*randn(3,1);   %  White noise (deg/s)

WMeas = wb + gyroscope_bias + gyroscope_white_noise;

end
