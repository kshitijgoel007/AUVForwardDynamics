function [ vel_meas ] = dvl_model(vel_bf , wb )
%DVL_MODEL returns measured velocity in dvl frame (1 x 3)
%   true_vel : velocity of c.g in body frame (1 x 3)

global DVL_to_bf;
global dvl_location;
global r_cg;

d = [(dvl_location(1) -  r_cg(1));
     (dvl_location(2) -  r_cg(2));
     (dvl_location(3) -  r_cg(3))];

vel_i_meas = vel_bf + cross(wb_dot,d);

% Transform vel_bf to dvl frame
vel_i_meas = DVL_to_bf'*vel_i_meas';

% velocity model
vel_meas = vel_i_meas + randn(1,3);
vel_meas = vel_meas';
end

