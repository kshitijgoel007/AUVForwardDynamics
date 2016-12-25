function [ p_meas ] = pSensor_model( height )
%PSENSOR_MODEL returns measured pressure at given height
%   height : Height of c.g from bottom

global density; % density of water

p_meas = density*9.8*height + randn;

end

