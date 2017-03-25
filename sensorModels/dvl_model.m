function [ vel_meas ] = dvl_model(vel_bf, wb )
%DVL_MODEL returns measured velocity in dvl frame [m/s]
%
% Inputs :
%   vel_bf : velocity of c.g in body frame [u; v; w] [m/s]
%   wb : Body rates, [p; q; r] expressed in body-frame [rad/s]

global DVL_to_body;
global d_DVL;
global dvl_noise_density;
global tinc;

vel_i_meas = vel_bf + cross(wb,d_DVL); % TODO : check correctness

% Transform vel_bf to dvl frame
vel_i_meas = DVL_to_body'*vel_i_meas;

% Noise density
dvl_noise_density_d = dvl_noise_density*(1/sqrt(tinc));

% velocity model
vel_meas = vel_i_meas + dvl_noise_density_d*randn(3,1);
end
