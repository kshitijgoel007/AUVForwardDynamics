
close all;

addpath('utils');
 
u = 1;
p = 0;
tspan = [0 15];

X_initial_s = [0; 0; 0; 0];%v,r,del_r,psi
X_initial_h = [0; 0; 0; 0];%w,q,del_s,theta
%F_ext = [ 0; 0; 0; 0];

  
  
  for i=1:1
       
  prompt = 'input x-coordinate:';
  ansx = input(prompt);
  prompt = 'input y-coordinate:';
  ansy = input(prompt);
  prompt = 'input z-coordinate:';
  ansz = input(prompt);
  prompt = ('Case number : ');
  caseNo = input(prompt);
  
  X_initial_s(4,1) = atan(ansy/ansx);
  X_initial_h(4,1) = atan(ansz/sqrt(ansx*ansx + ansy*ansy));
  pos_init = [0 0 0];
  % Solving using 1st Order Euler
    %X(k+1) = X(k) +dX(k)*dk
  dt = .1;
  T = 0:.1:5;      
  ord_defl = [0;0;0;0];
 
  X = [1; X_initial_s(1); X_initial_h(1);...
       0; X_initial_h(2); X_initial_s(2);...
       0; 0;0;...
       0; X_initial_h(4); X_initial_s(4);...
       X_initial_h(3); 0; 0; X_initial_s(3);...
       0;0 ];  
      
      X_euler_s =X_initial_s;
      X_euler_h =X_initial_h;
      y(1,:) = X_initial_s;
      z(1,:) = X_initial_h;
      k(1,:) = pos_init;
      M(:,1) = X;
      
      for j=2:length(T)
           
           DX = forwarddynamics2(T(j),X,ord_defl,caseNo); 
           temp = X + DX*dt;
           X = temp;
           M(:,j) = X;
      
      end
      
      
      

      
%       
%          figure;
%        subplot(4,1,1);plot(T,y(:,1)); xlabel('time');ylabel('v');
% subplot(4,1,2);plot(T,y(:,2)); xlabel('time');ylabel('r');
% subplot(4,1,3);plot(T,y(:,3)); xlabel('time');ylabel('\delta_r');
% subplot(4,1,4);plot(T,y(:,4)); xlabel('time');ylabel('\psi'); 
%        
%        figure;
%        subplot(4,1,1);plot(T,z(:,1)); xlabel('time');ylabel('w');
% subplot(4,1,2);plot(T,z(:,2)); xlabel('time');ylabel('q');
% subplot(4,1,3);plot(T,z(:,3)); xlabel('time');ylabel('\delta_s');
% subplot(4,1,4);plot(T,z(:,4)); xlabel('time');ylabel('\theta'); 

% figure;
% subplot(3,1,1);plot(T,u); xlabel('time');ylabel('u');
% subplot(3,1,2);plot(T,y(:,1)); xlabel('time');ylabel('v');
% subplot(3,1,3);plot(T,z(:,1)); xlabel('time');ylabel('w');
% 
% figure;
% subplot(2,1,1);plot(T,y(:,3)); xlabel('time');ylabel('\delta_r');
% subplot(2,1,2);plot(T,y(:,4)); xlabel('time');ylabel('\psi'); 
%        
% figure;
% subplot(3,1,1);plot(T,p); xlabel('time');ylabel('p');
% subplot(3,1,1);plot(T,z(:,2)); xlabel('time');ylabel('q');
% subplot(3,1,2);plot(T,y(:,2)); xlabel('time');ylabel('r');
% 
% figure;
% subplot(2,1,1);plot(T,z(:,3)); xlabel('time');ylabel('\delta_s');
% subplot(2,1,2);plot(T,z(:,4)); xlabel('time');ylabel('\theta'); 
        
  end
 
disp(X);