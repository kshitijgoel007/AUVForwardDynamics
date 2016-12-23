clc
clear all

%% setup
addpath(genpath('datapoints'));
addpath(genpath('accelerometer_model.m'));

%% load auv parameters as global params.
s = load('datapoints.mat');
n = fieldnames(s);
for k = 1:length(n)
    eval(sprintf('global %s; %s=s.%s;',n{k},n{k},n{k}));
%     eval(sprintf('global %s;',n{k}));
end
clear s;

 
%% load simulation parameters
A = csvread('parametersForSimulator.csv',1);

t = A(:,1);
t0 = t(1); tf = t(length(t));
tinc = t(2) - t(1);

omega_bf = A(:,6:8);
omega_bf_dot = A(:,15:17);
velocity_bf = A(:,3:5);
accel_bf = A(:,12:14);

accel_bf_meas = zeros(size(accel_bf,1), 1+3); % [t, accel_meas]
omega_bf_meas = zeros(size(omega_bf,1), 1+3); % [t, omega_meas]

%% Loop
for i = 1:length(t)
   accel_bf_meas(i,:) = [(i-1)*tinc , accelerometer_model(omega_bf(i,:)', omega_bf_dot(i,:)', accel_bf(i,:)', tinc)'];
   omega_bf_meas(i,:) = [(i-1)*tinc , gyro_model(omega_bf(i,:)', tinc)'];
end

figure
plot(t,accel_bf); 
hold on;
plot(t,accel_bf_meas(:,2:4));
legend('a_{x true}','a_{y true}','a_{z true}','a_{x measured}','a_{y measured}','a_{z measured}');
xlabel('time (sec)');
ylabel('linear acc (m/s^2)');
hold off;

figure
plot(t,omega_bf*180/3.14); 
hold on;
plot(t,omega_bf_meas(:,2:4));
legend(' p_{true}','q_{true}','r_{true}','p_{measured}','q_{measured}','r_{measured}');
xlabel('time (sec)');
ylabel('Angular vel. (deg/s)');
hold off;

