function [ R ] = rot( theta, axis )
%ROT returns the rotation matrix about "axis"
%   Anti-clockwise theta is +ve
%   where theta is in degrees

if strcmp(axis,'z')
    R = [cosd(theta) sind(theta) 0;
        -sind(theta) cosd(theta) 0;
         0           0           1];
     
elseif strcmp(axis,'y')
    R = [cosd(theta) 0  -sind(theta);
         0           1             0;
         sind(theta) 0   cosd(theta);];
    
elseif strcmp(axis,'x')
    R = [1           0            0;
         0  cosd(theta) sind(theta);
         0 -sind(theta) cosd(theta);];
end

end

