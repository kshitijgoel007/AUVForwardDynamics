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
persistent prevWb;

% earth_rate_mat = skew(R_i2t*earth_rate); % earth_rate in tangent frame.
% R_t2I = IMU_to_body'*DCM(X(4:6)); % DOUBT : it is R_itoI ?

g_cap = [0 0 1]'*gravity; % TODO : change gravity - earth_rate_mat*earth_rate_mat*X(1:3); % earth gravity in tangent frame.
R_t2b = DCM(X(4:6));
x_dot = zeros(9,1);

wb = IMU_to_body*U(4:6); % Body rate

if isempty(prevWb)
    prevWb = wb;
end
wb_dot = (wb - prevWb)/tinc;
prevWb = wb;

% dot{r_{b/t}^{t}} %
x_dot(1:3) = R_t2b' * X(7:9); 

% theta %
x_dot(4:6) = euler_to_bodyRates(X(4:6), -1) * wb; % - R_t2I*(R_i2t*earth_rate));

% dot{v_{b/t}^{b}} %
x_dot(7:9) = IMU_to_body*U(1:3) + R_t2b*g_cap ...
             - cross(wb, cross(wb, d_IMU)) ...
             - cross(wb_dot, d_IMU);
             % - cross(U(4:6), X(7:9))  - R_t2I*earth_rate_mat*R_t2I*X(7:9)  
end
