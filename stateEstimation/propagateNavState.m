function [ newState ] = propagateNavState( prevState,IMUInput, tinc )
% PROPAGATE_NAV_STATE, calculated navigation states at t+1 using data at t and xdot

predState = prevState + NavStateDotWithoutBias(prevState,IMUInput)*tinc;
corrState = prevState + NavStateDotWithoutBias(predState,IMUInput)*tinc;
newState = (predState + corrState)/2;

end

