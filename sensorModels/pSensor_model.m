function [ p_meas ] = pSensor_model( depth )
%PSENSOR_MODEL returns measured pressure at given height
%   Depht : Depth of c.g from surface of water

global density; % density of water

p_meas = density*9.8*depth + 0.005*randn;

end

