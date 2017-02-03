% Reference for euler angles and velocity, position integeration
% http://www.chrobotics.com/library/understanding-euler-angles
% http://www.chrobotics.com/library/accel-position-velocity


clc
clear all
close all;
%% setup
addpath(genpath('datapoints'));
addpath(genpath('sensorModels'));
addpath(genpath('helperFunctions'));
addpath(genpath('ukf'));

%% load auv parameters as global params.
s = load('datapoints.mat');
n = fieldnames(s);
for k = 1:length(n)
    eval(sprintf('global %s; %s=s.%s;',n{k},n{k},n{k}));
end
clear s;
global d;
d = [(accelerometer_location(1) -  r_cg(1));
     (accelerometer_location(2) -  r_cg(2));
     (accelerometer_location(3) -  r_cg(3))];
 
%% load simulation parameters

A = csvread('MatrixForSensor.csv',1);
vel_bf = A(:,2:4)'; % m/s
omega_bf = A(:,5:7)'; % rad/s
position_in = A(:,8:10)'; % m
euler_angle = A(:,11:13)'; % rad
accel_bf = A(:,14:16)'; % m/s2
omega_bf_dot = A(:,17:19)'; % rad/s2
X_true = [position_in;
          euler_angle;
          vel_bf;];

% time parameters
t = A(:,1);
t0 = t(1); tf = t(length(t));
tinc = t(2) - t(1);

IMUaccel_bf_meas = zeros(size(accel_bf)); % [accel_meas expressed in sensor frame] of contact point at IMU
Gyro_omega_bf_meas = zeros(size(omega_bf)); % [omega_meas expressed in sensor frame] of contact point at IMU

accel_bias = zeros(size(accel_bf));
omega_bias = zeros(size(omega_bf));

g = [0 0 1]'*gravity;

ac_bias = [0 0 0]';
omeg_bias = [0 0 0]';

%% UKF parameters

sigma_a = accelerometer_noise_density*(1/sqrt(tinc));
sigma_g = gyroscope_noise_density*(1/sqrt(tinc));
sigma_apg = sigma_a + sigma_g; % sigma a plus g;
% P = diag(ones(9,1));
P = diag([0.05,0.05,0.05, 0.05,0.05,0.05, 0.05,0.05,0.05]);
Q = diag([0,0,0, sigma_g, sigma_g, sigma_g, sigma_apg, sigma_apg, sigma_apg]);
R = diag([0.5, 0.00087, 0.05]);


% Estimated states [ pos, euler_angles, velocity] %%
X_est = zeros(size(X_true));
P_est = zeros(size(X_true,2),9,9);
yMeas = zeros(3, size(X_true,2));
e = zeros(3,size(X_true,2));

% Initial conditions
X_est(7:9,1) = [1,0,0]';
init_state_guess = X_est(:,1);
P_est(1,:,:) = P;


ukf = extendedKalmanFilter(@propagateNavState,... % State transition function
                           @getSensorData,... % Measurement function
                           init_state_guess, ...
                           'HasAdditiveMeasurementNoise', true);
ukf.MeasurementNoise = R;
ukf.ProcessNoise = Q;

%% Loop
for i = 1:length(t)
   % measured accel and omega in sensor frame    
   [IMUaccel_bf_meas(:,i), accel_bias(:,i)] = accelerometer_model(DCM(euler_angle(:,i)),omega_bf(:,i), omega_bf_dot(:,i), accel_bf(:,i), ac_bias, tinc);
   [Gyro_omega_bf_meas(:,i), omega_bias(:,i)] = gyro_model(omega_bf(:,i), omeg_bias, tinc);
   accel_bias = accel_bias(:,i); % prev. values of bias
   omega_bias = omega_bias(:,i);
   
   U = [IMUaccel_bf_meas(:,i); Gyro_omega_bf_meas(:,i)];
   
   yMeas(:,i) = dvl_model(vel_bf(:,i), omega_bf(:,i));
%    getSensorData(X_est(:,i))
   
    if (i < length(t)) 
        if(rem(i,10) == 0)
            [X_est(:,i+1),P_est(i+1,:,:)] = correct(ukf, yMeas(:,i));        
        end
        [X_est(:,i+1),P_est(i+1,:,:)] = predict(ukf, U, tinc);
        
    end
    
   e(:,i) = yMeas(:,i) - getSensorData(X_est(:,i));
   
   fprintf('t : %d \n',i*tinc);
    
    
end

figure
plot(t,position_in); 
hold on;
plot(t,X_est(1:3,:));
legend('x_{true}','y_{true}','z_{true}','x_{est}','y_{est}','z_{est}');
xlabel('time (sec)');
ylabel('Position (m)');
hold off;


figure
plot(t,euler_angle*180/3.14); 
hold on;
plot(t,X_est(4:6,:)*180/3.14);
I = legend('$\phi_{true}$','$\theta_{true}$','$\psi_{true}$','$\phi_{est}$','$\theta_{est}$','$\psi_{est}$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('degrees');
hold off;

figure
plot(t,vel_bf); 
hold on;
plot(t,X_est(7:9,:));
legend('v_{x true}','v_{y true}','v_{z true}','v_{x est}','v_{y est}','v_{z est}');
xlabel('time (sec)');
ylabel('Linear vel. (m/s)');
hold off;


figure
plot(t,e);
legend('e_{vx}','e_{vy}','e_{vz}');
xlabel('time (sec)');
ylabel('Linear vel. (m/s)');
hold off;

eStates = (X_true - X_est)';
timeVector = t;

figure();
subplot(3,1,1);
plot(timeVector,eStates(:,1),...               % Error for the first state
    timeVector, sqrt(P_est(:,1,1)),'r', ... % 1-sigma upper-bound
    timeVector, -sqrt(P_est(:,1,1)),'r');   % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for r_{x}');
title('State estimation errors');
subplot(3,1,2);
plot(timeVector,eStates(:,2),...               % Error for the second state
    timeVector,sqrt(P_est(:,2,2)),'r', ...  % 1-sigma upper-bound
    timeVector,-sqrt(P_est(:,2,2)),'r');    % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for r_{y}');
legend('State estimate','1-sigma uncertainty bound',...
    'Location','Best');
subplot(3,1,3);
plot(timeVector,eStates(:,3),...               % Error for the third state
    timeVector,sqrt(P_est(:,3,3)),'r', ...  % 1-sigma upper-bound
    timeVector,-sqrt(P_est(:,3,3)),'r');    % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for state r_{z}');
legend('State estimate','1-sigma uncertainty bound',...
    'Location','Best');

figure();
subplot(3,1,1);
plot(timeVector,eStates(:,4),...               % Error for the fourth state
    timeVector, sqrt(P_est(:,4,4)),'r', ... % 1-sigma upper-bound
    timeVector, -sqrt(P_est(:,4,4)),'r');   % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for \phi_{x}');
title('State estimation errors');
subplot(3,1,2);
plot(timeVector,eStates(:,5),...               % Error for the fifth state
    timeVector,sqrt(P_est(:,5,5)),'r', ...  % 1-sigma upper-bound
    timeVector,-sqrt(P_est(:,5,5)),'r');    % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for \theta_{y}');
legend('State estimate','1-sigma uncertainty bound',...
    'Location','Best');
subplot(3,1,3);
plot(timeVector,eStates(:,6),...               % Error for the sixth state
    timeVector,sqrt(P_est(:,6,6)),'r', ...  % 1-sigma upper-bound
    timeVector,-sqrt(P_est(:,6,6)),'r');    % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for state \psi_{z}');
legend('State estimate','1-sigma uncertainty bound',...
    'Location','Best');

figure();
subplot(3,1,1);
plot(timeVector,eStates(:,7),...               % Error for the seventh state
    timeVector, sqrt(P_est(:,7,7)),'r', ... % 1-sigma upper-bound
    timeVector, -sqrt(P_est(:,7,7)),'r');   % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for v_{x}');
title('State estimation errors');
subplot(3,1,2);
plot(timeVector,eStates(:,8),...               % Error for the eith state
    timeVector,sqrt(P_est(:,8,8)),'r', ...  % 1-sigma upper-bound
    timeVector,-sqrt(P_est(:,8,8)),'r');    % 1-sigma lower-bound
xlabel('Time [s]'); 
ylabel('Error for v_{y}');
legend('State estimate','1-sigma uncertainty bound',...
    'Location','Best');
subplot(3,1,3);
plot(timeVector,eStates(:,9),...               % Error for the ninth state
    timeVector,sqrt(P_est(:,9,9)),'r', ...  % 1-sigma upper-bound
    timeVector,-sqrt(P_est(:,9,9)),'r');    % 1-sigma lower-bound
xlabel('Time [s]');
ylabel('Error for state v_{z}');
legend('State estimate','1-sigma uncertainty bound',...
    'Location','Best');


% figure
% plot(t,IMUaccel_bf_meas);
% legend('udot_{measured}','vdot_{measured}','wdot_{measured}');
% xlabel('time (sec)');
% ylabel('Acc. (m/s2)');
% hold off;
% 
% figure
% plot(t,Gyro_omega_bf_meas);
% legend('p_{measured}','q_{measured}','r_{measured}');
% xlabel('time (sec)');
% ylabel('Body rate (rad/s)');
% hold off;

%}