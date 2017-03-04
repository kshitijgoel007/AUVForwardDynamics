function [ U ] = getEKFinputs(euler_angle, omega_bf, omega_bf_dot, accel_bf, tinc)
% GET_KF_INPUTS returns input of EKF prediction step
%
% Inputs :
%   euler_angle : [phi, theta, psi]' [rad]
%   omega_bf : [p, q, r]'   [rad/s]
%   omega_bf_dot : [p_dot, q_dot, r_dot]' [rad/s2]
%   accel_bf : [u, v, w]' [m/s2]
%   tinc : Sec
%
%   U is IMUInput : [ax_meas; ay_meas; az_meas; gx_meas; gy_meas; gz_meas]; in respective IMU frames [ m/s2, rad/s]
%                   are accelerometer and gyro data in m/s2 and rad/s in their resp. frames

persistent omega_bias;
persistent accel_bias;

if isempty(omega_bias)
    omega_bias = zeros(3,1);
end

if isempty(accel_bias)
    accel_bias = zeros(3,1);
end
                                                 
% Get measured acceleration and Angular Vel. in respective sensor frames
[IMUaccel_bf_meas, accel_bias] = accelerometer_model(DCM(euler_angle), ... % R_inertialF_to_bodyF
                                                     omega_bf, ...         % Body rates
                                                     omega_bf_dot, ...     % Derivative of Body rates
                                                     accel_bf, ...         % u_dot
                                                     accel_bias, tinc);    % Prev bias, dt

[Gyro_omega_bf_meas, omega_bias] = gyro_model(omega_bf, ... % Body rate
                                              omega_bias, tinc);  % Prev bias, dt

U = [  IMUaccel_bf_meas;
       Gyro_omega_bf_meas
    ];

end
