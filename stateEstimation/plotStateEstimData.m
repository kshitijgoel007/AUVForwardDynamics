function plotStateEstimData(t,X_est, Y)
figure
plot(t,Y(:,8:1:10));
hold on;
    plot(t,X_est(1:1:3,:));
legend('x_{true}','y_{true}','z_{true}','x_{est}','y_{est}','z_{est}');
xlabel('time (sec)');
ylabel('Position (m)');
hold off;


figure
plot(t,Y(:,11:1:13)); %atan2(sin(euler_angle), cos(euler_angle))*180/3.14);
hold on;
plot(t',X_est(4:6,:)*180/3.14);
I = legend('$\phi_{true}$','$\theta_{true}$','$\psi_{true}$','$\phi_{est}$','$\theta_{est}$','$\psi_{est}$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('degrees');
hold off;

figure
plot(t,Y(:,2:1:4));
hold on;
plot(t,X_est(7:9,:));
legend('v_{x true}','v_{y true}','v_{z true}','v_{x est}','v_{y est}','v_{z est}');
xlabel('time (sec)');
ylabel('Linear vel. (m/s)');
hold off;

% figure();
% subplot(3,1,1);
% plot(timeVector,eStates(:,1),...               % Error for the first state
%     timeVector, sqrt(P_est(:,1,1)),'r', ... % 1-sigma upper-bound
%     timeVector, -sqrt(P_est(:,1,1)),'r');   % 1-sigma lower-bound
% xlabel('Time [s]');
% ylabel('Error for r_{x}');
% title('State estimation errors');
% subplot(3,1,2);
% plot(timeVector,eStates(:,2),...               % Error for the second state
%     timeVector,sqrt(P_est(:,2,2)),'r', ...  % 1-sigma upper-bound
%     timeVector,-sqrt(P_est(:,2,2)),'r');    % 1-sigma lower-bound
% xlabel('Time [s]');
% ylabel('Error for r_{y}');
% legend('State estimate','1-sigma uncertainty bound',...
%     'Location','Best');
% subplot(3,1,3);
% plot(timeVector,eStates(:,3),...               % Error for the third state
%     timeVector,sqrt(P_est(:,3,3)),'r', ...  % 1-sigma upper-bound
%     timeVector,-sqrt(P_est(:,3,3)),'r');    % 1-sigma lower-bound
% xlabel('Time [s]');
% ylabel('Error for state r_{z}');
% legend('State estimate','1-sigma uncertainty bound',...
%     'Location','Best');
% 
% figure();
% subplot(3,1,1);
% plot(timeVector,eStates(:,4),...               % Error for the fourth state
%     timeVector, sqrt(P_est(:,4,4)),'r', ... % 1-sigma upper-bound
%     timeVector, -sqrt(P_est(:,4,4)),'r');   % 1-sigma lower-bound
% xlabel('Time [s]');
% ylabel('Error for \phi_{x}');
% title('State estimation errors');
% subplot(3,1,2);
% plot(timeVector,eStates(:,5),...               % Error for the fifth state
%     timeVector,sqrt(P_est(:,5,5)),'r', ...  % 1-sigma upper-bound
%     timeVector,-sqrt(P_est(:,5,5)),'r');    % 1-sigma lower-bound
% xlabel('Time [s]');
% ylabel('Error for \theta_{y}');
% legend('State estimate','1-sigma uncertainty bound',...
%     'Location','Best');
% subplot(3,1,3);
% plot(timeVector,eStates(:,6),...               % Error for the sixth state
%     timeVector,sqrt(P_est(:,6,6)),'r', ...  % 1-sigma upper-bound
%     timeVector,-sqrt(P_est(:,6,6)),'r');    % 1-sigma lower-bound
% xlabel('Time [s]');
% ylabel('Error for state \psi_{z}');
% legend('State estimate','1-sigma uncertainty bound',...
%     'Location','Best');
% 
% figure();
% subplot(3,1,1);
% plot(timeVector,eStates(:,7),...               % Error for the seventh state
%     timeVector, sqrt(P_est(:,7,7)),'r', ... % 1-sigma upper-bound
%     timeVector, -sqrt(P_est(:,7,7)),'r');   % 1-sigma lower-bound
% xlabel('Time [s]');
% ylabel('Error for v_{x}');
% title('State estimation errors');
% subplot(3,1,2);
% plot(timeVector,eStates(:,8),...               % Error for the eith state
%     timeVector,sqrt(P_est(:,8,8)),'r', ...  % 1-sigma upper-bound
%     timeVector,-sqrt(P_est(:,8,8)),'r');    % 1-sigma lower-bound
% xlabel('Time [s]');
% ylabel('Error for v_{y}');
% legend('State estimate','1-sigma uncertainty bound',...
%     'Location','Best');
% subplot(3,1,3);
% plot(timeVector,eStates(:,9),...               % Error for the ninth state
%     timeVector,sqrt(P_est(:,9,9)),'r', ...  % 1-sigma upper-bound
%     timeVector,-sqrt(P_est(:,9,9)),'r');    % 1-sigma lower-bound
% xlabel('Time [s]');
% ylabel('Error for state v_{z}');
% legend('State estimate','1-sigma uncertainty bound',...
%     'Location','Best');
% 
% 
% 
figure
plot3(Y(:,8), Y(:,9), Y(:,10));
hold on
x = X_est(1:3,:);
plot3(x(1,:),x(2,:),x(3,:));
hold off;
end