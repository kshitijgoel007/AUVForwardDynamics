function [ x_dot ] = NavStateDotWithoutBias( X , U, tinc)
% NAV_STATE_DOT_WITHOUT_BIAS calculates estimated value x_dot = f(x_cap,u) and returns x_dot_cap
%
% Inputs :
%   X : [x; y; z; phi; theta; psi; v_bx; v_by; v_bz]
%           Note that, x, y, z are expressed in tangent frame [m],
%                      v_bx, v_by, v_bz expressed in body fixed frame. [m/s]
%
%   U is IMUInput : [ax_meas; ay_meas; az_meas; gx_meas; gy_meas; gz_meas]; in respective IMU frames [ m/s2, rad/s]
%                   are accelerometer and gyro data in m/s2 and rad/s in their resp. frames

% Req. data : earth_rate in tangent frame, import skew, IMU_to_body

% global R_i2t; % Inertial to tangential frame.
% global earth_rate; % in i frame 3x1

global IMU_to_body;
global gravity;
global d_IMU;
global R_i2t;
global earth_rate;
persistent prevWb;
persistent wb_dot;

earth_rate_t = R_i2t*earth_rate; % earth_rate in tangent frame.

g_cap = [0 0 1]'*gravity;
R_t2b = DCM(X(4:6));
x_dot = zeros(9,1);

wb = IMU_to_body*U(4:6); % Body rate

if isempty(prevWb)
    prevWb = wb;
    wb_dot = [0 0 0]';
end

if(~isequal(wb, prevWb))
    wb_dot = (wb - prevWb)/tinc;
end

prevWb = wb;

% dot{r_{b/t}^{t}} %
x_dot(1:3) = R_t2b' * X(7:9);

% theta %
x_dot(4:6) = euler_to_bodyRates(X(4:6), -1) * wb; % - R_t2I*(R_i2t*earth_rate));

% dot{v_{b/t}^{b}} %
x_dot(7:9) = IMU_to_body*U(1:3) + R_t2b*g_cap ...
             - cross(wb_dot, d_IMU) ...
             - cross(wb, cross(wb, d_IMU)) ...
             - R_t2b*cross(earth_rate_t, R_t2b'*X(7:9)) ...
             - cross(wb, X(7:9)) ...
             - R_t2b*cross(earth_rate_t, cross(earth_rate_t, X(1:3) ) );
end
