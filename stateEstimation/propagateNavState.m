function [ newState ] = propagateNavState( prevState,IMUInput, tinc )
% PROPAGATE_NAV_STATE, calculated navigation states at t+1 using data at t and xdot

global count
% global input

% if (IMUInput == input)
% %     input
%     fprintf('Yes \n');
% else
% %     zeros(6,1)
%     fprintf('NO \n');
% end
predState = prevState + NavStateDotWithoutBias(prevState,IMUInput)*tinc;
corrState = prevState + NavStateDotWithoutBias(predState,IMUInput)*tinc;
newState = (predState + corrState)/2;
count = count + 1;
% if (count == 501)
%     pause;
% newState = predState;
end

