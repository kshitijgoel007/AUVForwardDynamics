clc
clear all

%% setup
addpath(genpath('datapoints'));
addpath(genpath('helperFunctions'));

%% load auv parameters as global params.
s = load('datapoints.mat');
n = fieldnames(s);
for k = 1:length(n)
    eval(sprintf('global %s; %s=s.%s;',n{k},n{k},n{k}));
end
clear s;
d = [(accelerometer_location(1) -  r_cg(1));
     (accelerometer_location(2) -  r_cg(2));
     (accelerometer_location(3) -  r_cg(3))];
 
%% load simulation parameters
A = csvread('parametersForSimulator.csv',1);

t = A(:,1);
t0 = t(1); tf = t(length(t));
tinc = t(2) - t(1);

omega_bf = A(:,6:8);
omega_bf_dot = A(:,15:17);
position_in = A(:,9:11);
vel_bf = A(:,3:5);
accel_bf = A(:,12:14);

accel_bf_meas = zeros(size(accel_bf,1), 3); % [accel_meas expressed in sensor frame] of contact point at IMU
omega_bf_meas = zeros(size(omega_bf,1), 3); % [omega_meas expressed in sensor frame] of contact point at IMU
accel_bias_vec = zeros(size(accel_bf,1), 3);
omega_bias_vec = zeros(size(omega_bf,1), 3);

accel_cg_bf_meas = zeros(size(accel_bf,1), 3); % [accel of c.g calculated as expressed in sensor frame w.r.t inertial frame]
vel_cg_bf = zeros(size(vel_bf,1),3);
position_cg_in = zeros(size(position_in,1), 3); % Position of c.g in inertial frame of ref.

accel_bias = [0 0 0];
omega_bias = [0 0 0];

%% Initial conditions
vel_cg_bf(1,:) = [1, 0, 0];

%% Loop
for i = 1:length(t)
   % measured accel and omega in sensor frame    
   [accel_bf_meas(i,:), accel_bias_vec(i,:)] = accelerometer_model(omega_bf(i,:)', omega_bf_dot(i,:)', accel_bf(i,:)', accel_bias', tinc);
   [omega_bf_meas(i,:), omega_bias_vec(i,:)] = gyro_model(omega_bf(i,:)', omega_bias', tinc);
   accel_bias = accel_bias_vec(i,1:3); % prev. values of bias
   omega_bias = omega_bias_vec(i,1:3);
   
   % Transform the measured values to body frame
   accel_bf_meas(i,:) = (IMU_to_body * accel_bf_meas(i,:)')'; % Acceleration of point where IMU is placed
   omega_bf_meas(i,:) = (IMU_to_body * omega_bf_meas(i,:)')';
   
   % Calculate the acceleration of c.gs
   % TODO : use measured wb and estimate wb_dot and then use them here.
   accel_cg_bf_meas(i,:) = accel_bf_meas(i,:) - cross(omega_bf(i,:)',cross(omega_bf(i,:)',d))' - cross(omega_bf_dot(i,:)',d)';
   
   % Calculate velocity [bf]
   if(i < length(t))
       vel_cg_bf(i+1,:) = vel_cg_bf(i,:) + accel_cg_bf_meas(i,:)*tinc;
   end
   
   % Calculate position [inertial frame]
   if(i < length(t))
      position_cg_in(i+1,:) = position_cg_in(i,:) + vel_cg_bf(i,:)*tinc; 
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
plot(t,omega_bf_meas);
legend(' p_{true}','q_{true}','r_{true}','p_{measured}','q_{measured}','r_{measured}');
xlabel('time (sec)');
ylabel('Angular vel. (deg/s)');
hold off;

figure
plot(t,vel_bf); 
hold on;
plot(t,vel_cg_bf);
legend('v_{x true}','v_{y true}','v_{z true}','v_{x measured}','v_{y measured}','v_{z measured}');
xlabel('time (sec)');
ylabel('Linear vel. (m/s)');
hold off;

figure
plot(t,position_in); 
hold on;
plot(t,position_cg_in);
legend('x_{true}','y_{true}','z_{true}','x_{calculated}','y_{calculated}','z_{calculated}');
xlabel('time (sec)');
ylabel('Position (m)');
hold off;

%{
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

