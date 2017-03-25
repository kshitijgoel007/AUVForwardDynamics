global accelerometer_location IMU_Accelerometer_SF_MA IMU_to_body;
global accelerometer_noise_density accelerometer_bias_instability accelerometer_VRW accel_corr_time 
global gyroscope_noise_density gyroscope_bias_instability gyroscope_ARW gyro_corr_time 


accelerometer_location = [-100; 20; 20] *1e-3; % [m]

IMU_Accelerometer_SF_MA = [ 0  0  0
                            0  0  0
                            0  0  0];
IMU_to_body = [ 1  0  0
                0  1  0
                0  0  1];

accelerometer_noise_density = 126*1e-6*gravity;      % 126 micro g/rt(Hz)  [(m/s2)/rt(Hz)] 1 sigma (continuous)
% accelerometer_noise_density_d = accelerometer_noise_density*(1/sqrt(tinc)) % white noise 1 sigma (discrete)
accelerometer_bias_instability = 0.023*1e-3*gravity;   % 0.023 milli g  [m/s2]
accelerometer_VRW = 0.063;      % Velocity random walk (m/s) per rt(sec)
accel_corr_time = 110; % sec

gyroscope_noise_density = 0.03*(pi/180);      % 0.03 (deg/sec)/rt(Hz)  [(rad/sec) per rt(Hz)] 1 sigma (continuous)
% gyroscope_noise_density_d = gyroscope_noise_density*(1/sqrt(tinc)) % white noise 1 sigma (discrete)
gyroscope_bias_instability = 10.08*(1.0/3600)*(pi/180);   % 10.08 deg/hr  (rad/sec) [Bias instability]
gyroscope_ARW = 1.5*(1.0/60)*(pi/180);      % Angular random walk, 1.5 deg/rt(hr)  [rad per rt(sec)]
gyro_corr_time = 270;
