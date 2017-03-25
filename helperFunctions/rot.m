function [ R ] = rot( theta, axis )
%ROT returns the rotation matrix about "axis"
%   Anti-clockwise theta is +ve
%   where theta is in degrees

if strcmp(axis,'z')
    R = [cos(theta) sin(theta) 0;
        -sin(theta) cos(theta) 0;
         0           0           1];

elseif strcmp(axis,'y')
    R = [cos(theta) 0  -sin(theta);
         0           1             0;
         sin(theta) 0   cos(theta);];

elseif strcmp(axis,'x')
    R = [1           0            0;
         0  cos(theta) sin(theta);
         0 -sin(theta) cos(theta);];
en

en
