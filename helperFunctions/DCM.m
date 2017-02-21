function [ R ] = DCM( euler_angles )
%DCM Direction cosine matrix, also inertial to body frame transformation
%matrix.
%   euler_angles = [phi, theta, psi] 1x3 , in degrees
%  or rpy = [roll, pitch, yaw]

phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

% can also be calculated
% Euler seq : z-y-x (psi, theta, phi)
%    R = rot(phi, 'x')*rot(theta, 'y')*rot(psi, 'z');
% rpy seq : x-y-z   (roll, pitch, yaw)
%    R = rot(roll,'x')*rot(pitch, 'y')*rot(yaw, 'z');
% hence phi ~ roll, theta ~ pitch, psi ~ yaw.

% expanded form
% R = [ ...
%  cos(theta)*cos(psi)                                  cos(theta)*sin(psi)                                -sin(theta);
% -cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi)  cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi) sin(phi)*cos(theta)
%  sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi) -sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi) cos(phi)*cos(theta)];

R = [ ...
 cos(theta)*cos(psi)                               cos(theta)*sin(psi)                              -sin(theta);
-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi)  cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi) sin(psi)*cos(theta)
 sin(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi) -sin(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi) sin(phi)*cos(theta)];


end
