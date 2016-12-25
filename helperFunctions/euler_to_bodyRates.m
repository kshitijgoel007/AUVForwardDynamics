function [ rot_matrix ] = euler_to_bodyRates(euler_angles , type)
%EULER_TO_BODYRATES Returns the rotation matrix for conversion from euler
%to body fixed rates.
%   euler_angles [ phi, theta, psi] 1 x 3 (in degrees)


phi = euler_angles(1);
theta = euler_angles(2);
psi = euler_angles(3);

% calculate the rotation matrix
if type == 1
    rot_matrix = [ 1         0           -sind(theta);
                   0  cosd(phi) sind(phi)*cosd(theta);
                   0 -sind(phi) cosd(phi)*cosd(theta);];
elseif type == -1
    rot_matrix = [ 1  sind(phi)*tand(theta)  cosd(phi)*tand(theta);
                   0              cosd(phi)             -sind(phi);
                   0  sind(phi)/cosd(theta)  cosd(phi)/cosd(theta);];
end

end

