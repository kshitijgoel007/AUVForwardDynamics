function x_dot_cap = NavStateDot(X,Y)
% NAV_STATE_DOT calculates estimated value x_dot = f(x_cap,u) and returns
% x_dot_cap.
% X : [x, y, z, phi, theta, psi, v_bx, v_by, v_bz, ba_x, ba_y, ba_z, bg_x, bg_y, bg_z]
% Note that, x, y, z are expressed in tangent frame,
%           v_bx, v_by, v_bz expressed in body fixed frame.
%           bias are in gyro and accel. frames respectively.
% Y : [ax, ay, az, rx, ry, rz]
%   are accelerometer and gyro data in m/s2 and rad/s in their resp. frames

% Req. data : earth_rate, import skew, IMU_to_body, alpha

earth_rate_mat = skew(earth_rate); % earth_rate in tangent frame.
g_cap = earth_gravity - earth_rate_mat*earth_rate_mat*X(1:3); % earth gravity in tangent frame.
R_t2b = DCM(X(4:6))';

% Compensate for acc. due to IMU at diff. position than bf origin.
omega_matrix_tangentf = R_t2b'*skew(IMU_to_body*(Y(4:6)-X(13:15)))*R_t2b;
omegadot_matrix_tangentf = R_t2b'*skew(alpha)*R_t2b; % req. alpha in body frame
acc_residual_IMU2bf = - R_t2b*omega_matrix_tangentf*omega_matrix_tangentf*R_t2b*pos_IMU ...
                      - R_t2b*omegadot_matrix_tangentf*R_t2b*pos_IMU;

x_dot_cap(1:3) = R_t2b * X(7:9); % dot{r_{b/t}^{t}}
x_dot_cap(4:6) = euler_to_bodyRates(X(4:6), -1) * (IMU_to_body*(Y(4:6) - X(13:15)) - Rt2p_cap*earth_rate); %theta
x_dot_cap(7:9) = IMU_to_body*(Y(1:3) - X(10:12)) + R_t2b * g_cap - R_t2b*earth_rate_mat*R_t2b*X(7:9) ...
               - cross(IMU_to_body*(Y(4:6)-X(13:15)), X(7:9)) + acc_residual_IMU2bf; % dot{v_{b/t}^{b}}
x_dot_cap(10:15) = 0; % bias
end
