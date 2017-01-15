function [ x_dot ] = NavStateDotWithoutBias( X , U)
% NAV_STATE_DOT_WITHOUT_BIAS calculates estimated value x_dot = f(x_cap,u) and returns
% x_dot_cap.
% X : [x, y, z, phi, theta, psi, v_bx, v_by, v_bz]
% Note that, x, y, z are expressed in tangent frame,
%           v_bx, v_by, v_bz expressed in body fixed frame.
% Y : [ax, ay, az, rx, ry, rz]
%   are accelerometer and gyro data in m/s2 and rad/s in their resp. frames

% Req. data : earth_rate in tangent frame, import skew, IMU_to_body

global earth_rate; % in i frame 3x1
global IMU_to_body;
global R_i2t; % Inertial to tangential frame.

earth_rate_mat = skew(R_i2t*earth_rate); % earth_rate in tangent frame.
% g_cap = earth_gravity; % TODO : change gravity - earth_rate_mat*earth_rate_mat*X(1:3); % earth gravity in tangent frame.
R_t2I = IMU_to_body'*DCM(X(4:6)); % DOUBT : it is R_itoI ?

x_dot(1:3) = R_t2I' * X(7:9); % dot{r_{b/t}^{t}}
x_dot(4:6) = euler_to_bodyRates(X(4:6), -1) * (U(4:6) - X(13:15) - R_t2I*(R_i2t*earth_rate)); %theta
x_dot(7:9) = U(1:3) + R_t2I * g_cap - R_t2I*earth_rate_mat*R_t2I*X(7:9) - cross(U(4:6), X(7:9)); % dot{v_{b/t}^{b}}

end

