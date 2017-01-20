% Reference for euler angles and velocity, position integeration
% http://www.chrobotics.com/library/understanding-euler-angles
% http://www.chrobotics.com/library/accel-position-velocity

clc
clear all
close all
%% setup
addpath(genpath('datapoints'));
addpath(genpath('helperFunctions'));

%% load auv parameters as global params.
s = load('datapoints.mat');
n = fieldnames(s);
for k = 1:length(n)
    eval(sprintf('global %s; %s=s.%s;',n{k},n{k},n{k}));
end
global d
clear s;
d = [(accelerometer_location(1) -  r_cg(1));
     (accelerometer_location(2) -  r_cg(2));
     (accelerometer_location(3) -  r_cg(3))];

%% load simulation parameters
%{
A = csvread('parametersForSimulator.csv',1);
omega_bf = A(:,6:8); % rad
omega_bf_dot = A(:,15:17); % rad/s
position_in = A(:,9:11); % m
vel_bf = A(:,3:5); % m/s
accel_bf = A(:,12:14); % m/s2
%}

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
euler_rates_cal = zeros(size(omega_bf,1), 3); % rad/s [calculated euler angular rates]
euler_angles_cal = zeros(size(omega_bf,1), 3); % rad  [calculated euler angles]
omega_bf_dot_cal = zeros(size(omega_bf,1),3); % rad/s2 [calculated angular acceleration]

accel_bias = zeros(size(accel_bf,1), 3);
omega_bias = zeros(size(omega_bf,1), 3);

g = [0 0 1]'*gravity;

accel_cg_bf_meas = zeros(size(accel_bf,1), 3); % [accel of c.g calculated as expressed in sensor frame w.r.t inertial frame]
vel_bf_meas = zeros(size(vel_bf,1),3); % [ velocity of c.g calculated using sensor data]
position_in_meas = zeros(size(position_in,1), 3); % Position of c.g in inertial frame of ref.

ac_bias = [0 0 0];
omeg_bias = [0 0 0];

X_est = zeros(9, size(accel_bf,1));
%% Initial conditions
vel_bf_meas(1,:) = [1, 0, 0];
X_est(7:9,1) = [1,0,0]';

%% Loop
for i = 1:length(t)
   % measured accel and omega in sensor frame
   [IMUaccel_bf_meas(i,:), accel_bias(i,:)] = accelerometer_model(DCM(euler_angle(i,:)), omega_bf(i,:)', omega_bf_dot(i,:)', accel_bf(i,:)', ac_bias', tinc);
   [Gyro_omega_bf_meas(i,:), omega_bias(i,:)] = gyro_model(omega_bf(i,:)', omeg_bias', tinc);
   accel_bias = accel_bias(i,1:3); % prev. values of bias
   omega_bias = omega_bias(i,1:3);

   U = [IMUaccel_bf_meas(i,:)'; Gyro_omega_bf_meas(i,:)'];

   % Transform the measured values to body frame
   IMUaccel_bf_meas(i,:) = (IMU_to_body * IMUaccel_bf_meas(i,:)')'; % Acceleration of point where IMU is placed
   Gyro_omega_bf_meas(i,:) = (IMU_to_body * Gyro_omega_bf_meas(i,:)')';

   % Calculate angular accl.
   if i >= 2
       omega_bf_dot_cal(i,:) = (Gyro_omega_bf_meas(i,:) - Gyro_omega_bf_meas(i-1,:))/tinc;
   end

   % Calculate the acceleration of c.g in body frame
   % TODO : use measured wb and estimate wb_dot and then use them here.
   accel_cg_bf_meas(i,:) = IMUaccel_bf_meas(i,:) + (DCM(euler_angles_cal(i,:))*g)' ...
                           - cross(Gyro_omega_bf_meas(i,:)',cross(Gyro_omega_bf_meas(i,:)',d))' ...
                           - cross(omega_bf_dot_cal(i,:)',d)';

   % Calculate velocity [bf]
   if(i < length(t))
       vel_bf_meas(i+1,:) = vel_bf_meas(i,:) + accel_cg_bf_meas(i,:)*tinc;
   end

   % Calculate euler angles
   if(i < length(t))
       euler_angles_cal(i+1,:) =  euler_angles_cal(i,:) + euler_rates_cal(i,:)*tinc;
   end

   % Calculate euler rates
   if(i < length(t))
       euler_rates_cal(i+1,:) = (euler_to_bodyRates(euler_angles_cal(i+1,:),-1)*Gyro_omega_bf_meas(i,:)')'; % -1 means for body fixed rates to euler rates
   end

   % Calculate position [inertial frame]
   if(i < length(t))
      position_in_meas(i+1,:) = position_in_meas(i,:) + ( DCM(euler_angles_cal(i,:))'*vel_bf_meas(i,:)')'*tinc;
   end

    if (i < length(t)) 
        X_est(:,i+1) = propagateNavState(X_est(:,i), U, tinc);
    end
end

figure
plot(t,accel_bf);
hold on;
plot(t,accel_cg_bf_meas);
legend('a_{x true}','a_{y true}','a_{z true}','a_{x measured}','a_{y measured}','a_{z measured}');
xlabel('time (sec)');
ylabel('linear acc (m/s^2)');
hold off;

figure
plot(t,omega_bf*180/3.14);
hold on;
plot(t,Gyro_omega_bf_meas*180/3.14);
legend(' p_{true}','q_{true}','r_{true}','p_{measured}','q_{measured}','r_{measured}');
xlabel('time (sec)');
ylabel('Angular vel. (deg/s)');
hold off;

% figure
% plot(t,omega_bf_dot*180/3.14);
% hold on;
% plot(t,omega_bf_dot_cal*180/pi);
% I = legend('$\dot{p}_{true}$','$\dot{q}_{true}$','$\dot{r}_{true}$' ...
%           ,'$\dot{p}_{meas}$','$\dot{q}_{meas}$','$\dot{r}_{meas}$');
% set(I,'interpreter','latex');
% xlabel('time (sec)');
% ylabel('Angular accel. (deg/s^2)');
% hold off;

figure
plot(t,vel_bf);
hold on;
plot(t,vel_bf_meas);
legend('v_{x true}','v_{y true}','v_{z true}','v_{x measured}','v_{y measured}','v_{z measured}');
xlabel('time (sec)');
ylabel('Linear vel. (m/s)');
hold off;

figure
plot(t,position_in);
hold on;
plot(t,position_in_meas);
legend('x_{true}','y_{true}','z_{true}','x_{calculated}','y_{calculated}','z_{calculated}');
xlabel('time (sec)');
ylabel('Position (m)');
hold off;

figure
plot(t,euler_angle*180/3.14);
hold on;
plot(t,euler_angles_cal*180/3.14);
I = legend('$\phi_{true}$','$\theta_{true}$','$\psi_{true}$','$\phi$','$\theta$','$\psi$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('degrees');
hold off;

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
plot(t,euler_rates_cal);
hold on;
plot(t,euler_angles_cal);
I = legend('$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','$\phi$','$\theta$','$\psi$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('deg and deg/s');
hold off;


figure
plot(t,accel_bias_vec);
legend('b_{ax}','b_{ay}','b_{az}');
xlabel('time (sec)');
ylabel('Linear accel. (m/s^2)');

figure
plot(t,omega_bias_vec);
legend('b{\omega x}','b{\omega y}','b{\omega z}');
xlabel('time (sec)');
ylabel('Angular vel. (deg/s)');
%}
