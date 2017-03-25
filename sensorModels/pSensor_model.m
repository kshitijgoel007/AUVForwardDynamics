function p_meas = pSensor_model( depth )
%PSENSOR_MODEL returns measured pressure at given height
%   depth : Depth of c.g from surface of water

global density; % density of water
global gravity;
global Patm;

p_meas = depth + 0.05*randn;
% p_meas = Patm + density*gravity*depth + 0*randn;

end

