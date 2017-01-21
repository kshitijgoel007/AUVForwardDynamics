function [ vel_meas ] = dvl_model(vel_bf, wb )
%DVL_MODEL returns measured velocity in dvl frame 
%   true_vel : velocity of c.g in body frame

global DVL_to_body;
global d;

vel_i_meas = vel_bf + cross(wb,d);

% Transform vel_bf to dvl frame
vel_i_meas = DVL_to_body'*vel_i_meas;

% velocity model
vel_meas = vel_i_meas + 0.05*randn(3,1);
end

