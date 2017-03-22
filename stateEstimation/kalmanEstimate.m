% Reference for euler angles and velocity, position integeration
% http://www.chrobotics.com/library/understanding-euler-angles
% http://www.chrobotics.com/library/accel-position-velocity


clc
clear all;
close all;

%% setup
addpath('datapoints');
addpath('sensorModels');
addpath('helperFunctions');

% sensors to use in prediciton step
PSENSOR = 0;
DVL = 1;

%% load simulation parameters & data
A = csvread('GoodSensordata30min.csv',1);
vel_bf = A(:,2:4)'; % m/s
omega_bf = A(:,5:7)'*pi/180; % rad/s
position_in = A(:,8:10)'; % m
euler_angle = A(:,11:13)'*pi/180; % rad
accel_bf = A(:,20:22)'; % m/s2
omega_bf_dot = A(:,23:25)'*pi/180; % rad/s2

X_true = [position_in;
          euler_angle;
          vel_bf;];

% time parameters
t = A(:,1);
tinc = t(2) - t(1);

%% load auv parameters as global params. [req. tinc]
AUVsensors;

if DVL
    EKFparams;
end

%% Storage data matrices
IMUaccel_bf_meas = zeros(size(accel_bf)); % [accel_meas expressed in sensor frame] of contact point at IMU
Gyro_omega_bf_meas = zeros(size(omega_bf)); % [omega_meas expressed in sensor frame] of contact point at IMU

accel_bias = zeros(size(accel_bf));
omega_bias = zeros(size(omega_bf));

g = [0 0 1]'*gravity;

ac_bias = [0 0 0]';
omeg_bias = [0 0 0]';

%% KF parameters

% Estimated states [ pos, euler_angles, velocity] %
X_est = zeros(size(X_true));
P_est = zeros(size(X_true,2),9,9);
yDVL = zeros(3, size(X_true,2));
yPsens = zeros(1, size(X_true,2));
e = zeros(4,size(X_true,2));

% Initial conditions
X_est(:,1) = X_true(:,1);
init_state_guess = X_est(:,1);
P_est(1,:,:) = P;


ekf = extendedKalmanFilter(@propagateNavState,... % State transition function
                           @getSensorData,... % Measurement function
                           init_state_guess);
ekf.MeasurementNoise = R;
ekf.ProcessNoise = Q;

%% Loop
for i = 1:length(t)
   
   % Get measured acceleration and Angular Vel. in respective sensor frames
   [IMUaccel_bf_meas(:,i), accel_bias(:,i)] = accelerometer_model(DCM(euler_angle(:,i)), ... % R_inertialF_to_bodyF
                                                                  omega_bf(:,i), ...         % Body rates
                                                                  omega_bf_dot(:,i), ...     % Derivative of Body rates
                                                                  accel_bf(:,i), ...         % u_dot
                                                                  ac_bias, tinc);            % Prev bias, dt
   
   [Gyro_omega_bf_meas(:,i), omega_bias(:,i)] = gyro_model(omega_bf(:,i), ... % Body rate
                                                           omeg_bias, tinc);  % Prev bias, dt
   
   % Update bias
   accel_bias = accel_bias(:,i);
   omega_bias = omega_bias(:,i);

   % Measured IMU data as INPUT, U in prediction step
   U = [  IMUaccel_bf_meas(:,i);
          Gyro_omega_bf_meas(:,i)
       ];
  
   % Get measurement data of DVL & Pressure sensor
   yDVL(1:3,i) = dvl_model(vel_bf(:,i), omega_bf(:,i));
   yPsens(1,i) = pSensor_model(position_in(3,i));

   if (i < length(t))
       
        % Residual in KF
        if DVL
            e(1:3,i) = yDVL(:,i) - getSensorData(X_est(:,i), 1);
             
            % DVL update available only at 1 HZ
            if(rem(i,10) == 0)
                
                % Correction step
                [X_est(:,i+1),P_est(i+1,:,:)] = correct(ekf, yDVL(:,i), 1);

            end
        end
        
        if PSENSOR
            e(4,i) = yPsens(:,i) - getSensorData(X_est(:,i), 2);
            
            % Correction 
            
            [X_est(:,i+1),P_est(i+1,:,:)] = correct(ekf, yPsens(:,i), 2);
        end
        
        

       
        
        % Prediction step
        [X_est(:,i+1),P_est(i+1,:,:)] = predict(ekf, U, tinc);

    end

   fprintf('t : %d \n',i*tinc);

end

eStates = (X_true - X_est)';
timeVector = t;

plotStateEstimData(timeVector, X_est, P_est, A);

% figure
% plot(t,e(1:3,:));
% legend('e_{vx}','e_{vy}','e_{vz}');
% xlabel('time (sec)');
% ylabel('Error in Linear vel.(est - meas) (m/s)');
% hold off;

% figure
% plot(t, e(4,:));
% legend('e_{rz}');
% xlabel('time (sec)');
% ylabel('Error in z (m)');


cd '../AUV/stateEstimation/NoiseCheck'
saveas(figure(1),'Position.jpg');
saveas(figure(2),'Euler.jpg');
saveas(figure(3),'Velocity.jpg');
saveas(figure(4),'UncertainityPos.jpg');
saveas(figure(5),'UncertainityEuler.jpg');
saveas(figure(6),'UncertainityVel.jpg');
saveas(figure(7),'Trajectory.jpg');
% saveas(figure(4),'Innovation.jpg');

cd '../../../AUVForwardDynamics'

% %{
% [xe,xeLags] = xcorr(e,'coeff'); % 'coeff': normalize by the value at zero lag
% % Only plot non-negative lags
% idx = xeLags>=0;
% figure();
% plot(xeLags(idx),xe(idx));
% xlabel('Lags');
% ylabel('Normalized correlation');
% title('Auto-correlation of residuals (innovation)');
% 
mean(eStates)
% [xeStates1,xeStatesLags1] = xcorr(eStates(:,1),'coeff'); % 'coeff': normalize by the value at zero lag
% [xeStates2,xeStatesLags2] = xcorr(eStates(:,2),'coeff'); % 'coeff'
% [xeStates3,xeStatesLags3] = xcorr(eStates(:,3),'coeff'); % 'coeff'
% [xeStates4,xeStatesLags4] = xcorr(eStates(:,4),'coeff'); % 'coeff'
% [xeStates5,xeStatesLags5] = xcorr(eStates(:,5),'coeff'); % 'coeff'
% [xeStates6,xeStatesLags6] = xcorr(eStates(:,6),'coeff'); % 'coeff'
% [xeStates7,xeStatesLags7] = xcorr(eStates(:,7),'coeff'); % 'coeff'
% [xeStates8,xeStatesLags8] = xcorr(eStates(:,8),'coeff'); % 'coeff'
% [xeStates9,xeStatesLags9] = xcorr(eStates(:,9),'coeff'); % 'coeff'


% Only plot non-negative lags
% idx = xeStatesLags1>=0;
% figure();
% subplot(3,1,1);
% plot(xeStatesLags1(idx),xeStates1(idx));
% xlabel('Lags');
% ylabel('For state 1');
% title('Normalized auto-correlation of state estimation error');
% subplot(3,1,2);
% plot(xeStatesLags2(idx),xeStates2(idx));
% xlabel('Lags');
% ylabel('For state 2');
% subplot(3,1,3);
% plot(xeStatesLags3(idx),xeStates2(idx));
% xlabel('Lags');
% ylabel('For state 3');
% 
% idx = xeStatesLags4>=0;
% figure();
% subplot(3,1,1);
% plot(xeStatesLags4(idx),xeStates4(idx));
% xlabel('Lags');
% ylabel('For state 4');
% title('Normalized auto-correlation of state estimation error');
% subplot(3,1,2);
% plot(xeStatesLags5(idx),xeStates5(idx));
% xlabel('Lags');
% ylabel('For state 5');
% subplot(3,1,3);
% plot(xeStatesLags6(idx),xeStates6(idx));
% xlabel('Lags');
% ylabel('For state 6');
% 
% idx = xeStatesLags7>=0;
% figure();
% subplot(3,1,1);
% plot(xeStatesLags7(idx),xeStates7(idx));
% xlabel('Lags');
% ylabel('For state 7');
% title('Normalized auto-correlation of state estimation error');
% subplot(3,1,2);
% plot(xeStatesLags8(idx),xeStates8(idx));
% xlabel('Lags');
% ylabel('For state 8');
% subplot(3,1,3);
% plot(xeStatesLags9(idx),xeStates9(idx));
% xlabel('Lags');
% ylabel('For state 9');

% figure
% plot(t,r_dot);
% legend('v_{x true}','v_{y true}','v_{z true}');
% xlabel('time (sec)');
% ylabel('Linear vel. (m/s)');
% 
% figure
% plot(t,r);
% legend('x_{true}','y_{true}','z_{true}');
% xlabel('time (sec)');
% ylabel('Position (m)');


% figure
% plot(t, accel_meas);
% legend('udot_{measured}','vdot_{measured}','wdot_{measured}');
% xlabel('time (sec)');
% ylabel('Acc. (m/s2)');
% hold off;

% figure
% plot(t,Gyro_omega_bf_meas);
% legend('p_{measured}','q_{measured}','r_{measured}');
% xlabel('time (sec)');
% ylabel('Body rate (rad/s)');
% hold off;

% figure
% plot(t,vel_bf);
% legend('v_{x true}','v_{y true}','v_{z true}');
% xlabel('time (sec)');
% ylabel('Linear vel. (m/s)');
% 
% figure
% plot(t,position_in);
% legend('x_{true}','y_{true}','z_{true}');
% xlabel('time (sec)');
% ylabel('Position (m)');
% 
% figure
% plot(t,omega_bf);
% legend('x_{true}','y_{true}','z_{true}');
% xlabel('time (sec)');
% ylabel('omega_bf (rad/s)');
% 
% figure
% plot(t,euler_angle);
% legend('x_{true}','y_{true}','z_{true}');
% xlabel('time (sec)');
% ylabel('euler angle ');

% figure
% plot(t, accel_bf);
% legend('x_{true}','y_{true}','z_{true}');
% xlabel('time (sec)');
% ylabel('acceleration (m/s^2)');

% figure
% plot(t, omega_bf_dot);
% legend('x_{true}','y_{true}','z_{true}');
% xlabel('time (sec)');
% ylabel('omega^{dot} (rad/s^2)');

%}
