% Reference for euler angles and velocity, position integeration
% http://www.chrobotics.com/library/understanding-euler-angles
% http://www.chrobotics.com/library/accel-position-velocity


function [X_est, P_est] = stateEstimation(A, tinc)
% Estimated states [ pos, euler_angles, velocity] %

%% Get these params from dynamics
vel_bf = A(2:4)'; % m/s
omega_bf = A(5:7)'; % rad/s
position_in = A(8:10)'; % m
euler_angle = A(11:13)'; % rad
accel_bf = A(20:22)'; % m/s2
omega_bf_dot = A(23:25)'; % rad/s2

t = A(1);
disp(t);
%% ekf parameters
persistent ekf;
global P Q R;
global DVL PSENSOR;

if isempty(ekf)
    init_state_guess = [position_in;
                        euler_angle;
                        vel_bf];
    P_est = P;
    ekf = extendedKalmanFilter(@propagateNavState,... % State transition function
                               @getSensorData,... % Measurement function
                               init_state_guess);
    ekf.MeasurementNoise = R;
    ekf.ProcessNoise = Q;
end
   % Measured IMU data as INPUT, U in prediction step
   U = getEKFinputs(euler_angle, omega_bf, omega_bf_dot, accel_bf, tinc);

   % Get measurement data of DVL
   yDVL = dvl_model(vel_bf, omega_bf);

        if DVL
            
            % DVL update available only at 1 HZ
            if(rem(t,1) == 0)
                % Correction step
                [X_est, P_est] = correct(ekf, yDVL, 1);
            end
            
        end
        
        
        % Prediction step
        [X_est, P_est] = predict(ekf, U, tinc);
end