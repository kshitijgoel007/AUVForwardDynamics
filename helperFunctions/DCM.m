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
R = [ ...
 cosd(theta)*cosd(psi)                                  cosd(theta)*sind(psi)                                -sind(theta);
-cosd(phi)*sind(psi) + sind(phi)*sind(theta)*cosd(psi)  cosd(phi)*cosd(psi) + sind(phi)*sind(theta)*sind(psi) sind(phi)*cosd(theta)
 sind(phi)*sind(psi) + cosd(phi)*sind(theta)*cosd(psi) -sind(phi)*cosd(psi) + cosd(phi)*sind(theta)*sind(psi) cosd(phi)*cosd(theta)];



end

