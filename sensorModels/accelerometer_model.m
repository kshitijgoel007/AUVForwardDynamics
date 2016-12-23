% Three axis Accelerometer model - [22-12-2016]

function a_meas = acceleration_measured(wb, wb_dot, Acc_true, tinc)


d = [(Accelerometer_Location(1) -  r_cg(1));
     (Accelerometer_Location(2) -  r_cg(2));
     (Accelerometer_Location(3) -  r_cg(3))];

Aimeas = Acc_true + cross(wb,cross(wb,d)) + cross(wb_dot,d);

% Accelerometer bias signal
% accelerometer_sig_beta = acceleration_random_walk*sqrt(tinc);
% accelerometer_beta += accelero_sig_beta*randn(3,1);
accelerometer_beta = accelerometer_bias_stability*randn(3,1);

% Accelerometer white noise signal
accelerometer_noise_density_d = accelerometer_noise_density*(1/sqrt(tinc));
accelerometer_white_noise = accelerometer_noise_density_d*randn(3,1);     %  White noise (m/s2)

a_meas = (eye(3,3)+ IMU_Accelerometer_SF_MA)*Aimeas + accelerometer_white_noise + accelerometer_beta;

end
