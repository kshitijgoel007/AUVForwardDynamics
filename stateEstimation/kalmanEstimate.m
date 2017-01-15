% Reference for euler angles and velocity, position integeration
% http://www.chrobotics.com/library/understanding-euler-angles
% http://www.chrobotics.com/library/accel-position-velocity

clc
clear all

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
euler_rates_cal = zeros(size(omega_bf,1), 3); % rad/s [calculated euler angular rates]
euler_angles_cal = zeros(size(omega_bf,1), 3); % rad  [calculated euler angles]
omega_bf_dot_cal = zeros(size(omega_bf,1),3); % rad/s2 [calculated angular acceleration]

accel_bias = zeros(size(accel_bf,1), 3);
omega_bias = zeros(size(omega_bf,1), 3);

accel_cg_bf_cal = zeros(size(accel_bf,1), 3); % [accel of c.g calculated as expressed in sensor frame w.r.t inertial frame]
vel_bf_cal = zeros(size(vel_bf,1),3); % [ velocity of c.g calculated using sensor data]
position_in_cal = zeros(size(position_in,1), 3); % Position of c.g in inertial frame of ref.

ac_bias = [0 0 0];
omeg_bias = [0 0 0];

% Estimated
X_est = zeros(9, size(accel_bf,1));
P_est = zeros(size(accel_bf,1),9,9);
% vel_bf_est = zeros(size(accel_bf,1), 3); % m/s
% position_tf_est = zeros(size(accel_bf,1), 3); % m
% euler_angle_est = zeros(size(accel_bf,1), 3); % 

%% UKF parameters
sigma_a = accelerometer_noise_density*(1/sqrt(tinc));
sigma_g = gyroscope_noise_density*(1/sqrt(tinc));
sigma_apg = sigma_a + sigma_g; % sigma a plus g;
Q = diag([0,0,0, sigma_g, sigma_g, sigma_g, sigma_apg, sigma_apg, sigma_apg]);
% R = diag([0.5, 0.00087, 0.05]);
P = diag([0.05,0.05,0.05, 0.05,0.05,0.05, 0.05,0.05,0.05]);
P_est(1,:,:) = P;

alpha = 0.1;
beta = 2;
kappa = 0; 
mat = 1;

f_func = @propagateNavState;
% h_func = @ukf_track_h;

%% Initial conditions
vel_bf_cal(1,:) = [1, 0, 0];

%% Loop
for i = 1:length(t)
   % measured accel and omega in sensor frame    
   [IMUaccel_bf_meas(i,:), accel_bias(i,:)] = accelerometer_model(omega_bf(i,:)', omega_bf_dot(i,:)', accel_bf(i,:)', ac_bias', tinc);
   [Gyro_omega_bf_meas(i,:), omega_bias(i,:)] = gyro_model(omega_bf(i,:)', omeg_bias', tinc);
   accel_bias = accel_bias(i,1:3); % prev. values of bias
   omega_bias = omega_bias(i,1:3);
   
   % Transform the measured values to body frame
   IMUaccel_bf_meas(i,:) = (IMU_to_body * IMUaccel_bf_meas(i,:)')'; % Acceleration of point where IMU is placed
   Gyro_omega_bf_meas(i,:) = (IMU_to_body * Gyro_omega_bf_meas(i,:)')';
   
   % Calculate angular velocity
   if i >= 2
       omega_bf_dot_cal(i,:) = (Gyro_omega_bf_meas(i,:) - Gyro_omega_bf_meas(i-1,:))/tinc;
   end
   
   % Calculate the acceleration of c.g in body frame
   % TODO : use measured wb and estimate wb_dot and then use them here.
   accel_cg_bf_cal(i,:) = IMUaccel_bf_meas(i,:) ...
                           - cross(Gyro_omega_bf_meas(i,:)',cross(Gyro_omega_bf_meas(i,:)',d))' ...
                           - cross(omega_bf_dot_cal(i,:)',d)';
   
   % Calculate velocity [bf]
   if(i < length(t))
       vel_bf_cal(i+1,:) = vel_bf_cal(i,:) + accel_cg_bf_cal(i,:)*tinc;
   end
   
   % Calculate euler angles
   if(i < length(t))
       euler_angles_cal(i+1,:) = euler_angles_cal(i,:) + euler_rates_cal(i,:)*tinc;
   end
   
   % Calculate euler rates
   if(i < length(t))
       euler_rates_cal(i+1,:) = (euler_to_bodyRates(euler_angles_cal(i+1,:),-1)*Gyro_omega_bf_meas(i,:)')'; % -1 means for body fixed rates to euler rates
   end
   
   % Calculate position [inertial frame]
   if(i < length(t))
      position_in_cal(i+1,:) = position_in_cal(i,:) + ( DCM(euler_angles_cal(i,:))'*vel_bf_cal(i,:)')'*tinc; 
   end

   [X_est(:,i+1), P] = ukf_predict1(X_est(:,i), P, f_func, Q, tinc, alpha, beta, kappa, mat);
   
%    Zpre = ukf_track_h(X);
%    [X, P] = ukf_update1(Xpre, Ppre, raIn(:,i), h_func, R, [], alpha, beta, kappa, mat);
%    xyOut(:,i) = X;

end

figure
plot(t,accel_bf); 
hold on;
plot(t,accel_cg_bf_cal);
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

figure
plot(t,omega_bf_dot*180/3.14); 
hold on;
plot(t,omega_bf_dot_cal*180/3.14);
I = legend('$\dot{p}_{true}$','$\dot{q}_{true}$','$\dot{r}_{true}$' ...
          ,'$\dot{p}_{meas}$','$\dot{q}_{meas}$','$\dot{r}_{meas}$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('Angular accel. (deg/s^2)');
hold off;

figure
plot(t,vel_bf); 
hold on;
plot(t,vel_bf_cal);
legend('v_{x true}','v_{y true}','v_{z true}','v_{x measured}','v_{y measured}','v_{z measured}');
xlabel('time (sec)');
ylabel('Linear vel. (m/s)');
hold off;

figure
plot(t,position_in); 
hold on;
plot(t,position_in_cal);
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

