function [ newState ] = propagateNavState( prevState,IMUInput, tinc )
% PROPAGATE_NAV_STATE, calculates navigation states X(t_(k+1)) using data x , xdot of prev. time t
%
% Inputs :
%   prevState : X(t_k) [9x1]
%   IMUInput : [ax_meas; ay_meas; az_meas; gx_meas; gy_meas; gz_meas]; in respective IMU frames [ m/s2, rad/s]
%   tinc : timestep [sec]

predState = prevState + NavStateDotWithoutBias(prevState, IMUInput, tinc)*tinc;
corrState = prevState + NavStateDotWithoutBias(predState, IMUInput, tinc)*tinc;
newState = (predState + corrState)/2;

end
