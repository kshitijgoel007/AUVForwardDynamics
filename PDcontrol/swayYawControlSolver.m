
close all;
clear all;

% mat file containing A and B matrices
   global a b;
   a=0; b=0;
   A_and_B_matrices;
 
  tspan = [0 5];

X_initial = [0.1; 0; 0; 0];
%F_ext = [ 0; 0; 0; 0];

% storing the a and b values corresponding to -ve eigenvalues of B 
  a_b_valuesArray = check_eigen_B(A,B);
  fname2=['a_b_array','.csv'] ;
  fid2=fopen(fname2,'w');
  dlmwrite(fname2,a_b_valuesArray);
  fclose(fid2) ;
  [r,c] = size(a_b_valuesArray);
  
  
  for i=1:1
    % setting a and b 
      a = a_b_valuesArray(90,1);
      b = a_b_valuesArray(90,2);
      
    % Solving using ODE45
      [T,y] = ode45( @swayYawControl, tspan, X_initial);
       
    % Solving using 1st Order Euler
    % X(k+1) = X(k) +dX(k)*dk
%       dt = .1;
%       T = 0:.1:5;
%       X_euler =X_initial;
%       y(1,:) = X_initial;
%       
%       for j=2:length(T)
%            DX=swayYawControl(T(j),X_euler);
%            temp=X_euler+DX*dt;
%            X_euler=temp;
%            y(j,:) = X_euler;
%       end
      
      
      
      
       figure;
       plot(T,y(:,1));
       xlabel('t');
       ylabel('v');
       
       
       figure;
       plot(T,y(:,2));
       xlabel('t');
       ylabel('r');
       
       figure;
       plot(T,y(:,3));
       xlabel('t');
       ylabel('del_r');
       
       figure;
       plot(T,y(:,4));
       xlabel('t');
       ylabel('si');
       
  end
 

