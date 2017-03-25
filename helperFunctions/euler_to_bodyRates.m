function [ rot_matrix ] = euler_to_bodyRates(euler_angles , type)
%EULER_TO_BODYRATES Returns the rotation matrix for conversion from euler
%to body fixed rates.
%   euler_angles [ phi, theta, psi] 1 x 3 (in degrees)
%   type = 1 , euler to body
%   type = -1, body to euler


phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

% calculate the rotation matrix
if type == 1
    rot_matrix = [ 1         0           -sin(theta);
                   0  cos(phi) sin(phi)*cos(theta);
                   0 -sin(phi) cos(phi)*cos(theta);];
elseif type == -1
    rot_matrix = [ 1  sin(phi)*tan(theta)  cos(phi)*tan(theta);
                   0              cos(phi)             -sin(phi);
                   0  sin(phi)/cos(theta)  cos(phi)/cos(theta);];
end

end
