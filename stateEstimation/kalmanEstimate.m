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
vel_bf = A(:,2:4); % m/s
omega_bf = A(:,5:7); % rad/s
position_in = A(:,8:10); % m
euler_angle = A(:,11:13); % rad
accel_bf = A(:,14:16); % m/s2
omega_bf_dot = A(:,17:19); % rad/s2

% time parameters
t = A(:,1);
t0 = t(1); tf = t(length(t));
tinc = t(2) - t(1);

IMUaccel_bf_meas = zeros(size(accel_bf,1), 3); % [accel_meas expressed in sensor frame] of contact point at IMU
Gyro_omega_bf_meas = zeros(size(omega_bf,1), 3); % [omega_meas expressed in sensor frame] of contact point at IMU

accel_bias = zeros(size(accel_bf,1), 3);
omega_bias = zeros(size(omega_bf,1), 3);

g = [0 0 1]'*gravity;

ac_bias = [0 0 0];
omeg_bias = [0 0 0];

% Estimated states [ pos, euler_angles, velocity] %%
X_est = zeros(9, size(accel_bf,1));
X_est(7:9,1) = [1,0,0]';
init_state_guess = X_est(:,1);
P_est = zeros(size(accel_bf,1),9,9);
yMeas = zeros(3, size(accel_bf,1));

%% UKF parameters
sigma_a = accelerometer_noise_density*(1/sqrt(tinc));
sigma_g = gyroscope_noise_density*(1/sqrt(tinc));
sigma_apg = sigma_a + sigma_g; % sigma a plus g;
P = diag(ones(9,1));
% P = diag([0.05,0.05,0.05, 0.05,0.05,0.05, 0.05,0.05,0.05]);
Q = diag([0,0,0, sigma_g, sigma_g, sigma_g, sigma_apg, sigma_apg, sigma_apg]);
R = diag([0.5, 0.00087, 0.05]);

alpha = 0.8; % Using complementary filtering instead of UKF
H = zeros(9,3);
H(7:9,:) = eye(3,3);

P_est(1,:,:) = P;
e = zeros(3,size(accel_bf,1));
ukf = extendedKalmanFilter(@propagateNavState,... % State transition function
                            @getSensorData,... % Measurement function
                            init_state_guess, ...
                            'HasAdditiveMeasurementNoise', true);
ukf.MeasurementNoise = R;
ukf.ProcessNoise = Q;

global count
global input
count = 0;
%% Loop
for i = 1:length(t)
   % measured accel and omega in sensor frame    
   [IMUaccel_bf_meas(i,:), accel_bias(i,:)] = accelerometer_model(DCM(euler_angle(i,:)),omega_bf(i,:)', omega_bf_dot(i,:)', accel_bf(i,:)', ac_bias', tinc);
   [Gyro_omega_bf_meas(i,:), omega_bias(i,:)] = gyro_model(omega_bf(i,:)', omeg_bias', tinc);
   accel_bias = accel_bias(i,1:3); % prev. values of bias
   omega_bias = omega_bias(i,1:3);
   
   U = [IMUaccel_bf_meas(i,:)'; Gyro_omega_bf_meas(i,:)'];
   input = U; %check predict
   
   yMeas(:,i) = getSensorData(X_est(:,i), U, vel_bf(i,:)', omega_bf(i,:)');
   
    if (i < length(t)) 
%         correct(ukf, yMeas(:,i), U, vel_bf(i,:)', omega_bf(i,:)');        
        [X_est(:,i+1),P_est(i+1,:,:)] = predict(ukf, U, tinc);
        
%         X_est(:,i+1) = ukf.State;
        X_est(7:9,i+1) = alpha*yMeas(:,i) + (1-alpha)*H'*X_est(:,i+1);
    end
    
%    e(:,i) = yMeas(:,i) - H'*propagateNavState(X_est(:,i), U ,tinc);
    
    
    
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

%{
figure
plot(t,yMeas);
legend('v_{x meas}','v_{y meas}','v_{z meas}');
xlabel('time (sec)');
ylabel('Linear vel. (m/s)');
hold off;


figure
plot(t,IMUaccel_bf_meas);
legend('udot_{measured}','vdot_{measured}','wdot_{measured}');
xlabel('time (sec)');
ylabel('Acc. (m/s2)');
hold off;

figure
plot(t,Gyro_omega_bf_meas);
legend('p_{measured}','q_{measured}','r_{measured}');
xlabel('time (sec)');
ylabel('Body rate (rad/s)');
hold off;

%}