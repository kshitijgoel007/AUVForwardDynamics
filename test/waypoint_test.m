
close all;
clc;
addpath('utils');
addpath('PDcontrol/divingControl');
addpath('PDcontrol/steeringControl');
addpath('Solvers');
 
X_initial_s = [0; 0; 0; 0];%v,r,del_r,psi
X_initial_h = [0; 0; 0; 0];%w,q,del_s,theta


  
       
  %prompt = 'input x-coordinate:';
  ansx = 1;%input(prompt);
%   prompt = 'input y-coordinate:';
%   ansy = input(prompt);
%   prompt = 'input z-coordinate:';
%   ansz = input(prompt);
%   prompt = 'solver?? euler(0)/RK4(1)';
   s = 0;%input(prompt);
  
%   yaw_d= atan(ansy/ansx);
%   pitch_d = atan(ansz/sqrt(ansx*ansx + ansy*ansy));
  caseNo = '15';
  
  tstart =0;
  tstep = .2;
  tend = tstep;
 
  % simulation running time
  runtime = 5 ;
  
  %frequency
  dt = .1;
  
  ord_defl = [0;0;0;0];
  
  X = init_state();% initialise everything to zero

  ctr =0;
  
  % LOGIC USED:
  % run all controllers for tstep sec, then check if goal is reached
  % and vary control input accordingly
  
  %proportional control gain
  Kp = 10;
  
  while( tend < runtime)
    
  if ansx>X(7),X(18) = Kp*(ansx-X(7));
  else X(18) = 2*Kp*(ansx-X(7));
  end
  % running PD control for tstep sec 
   tspan = tstart:dt:tend;
 
  if(s==0)
   Y(ctr+1:ctr+length(tspan),:) = eulerFirstOrder(@forwarddynamics2,tspan,X, ord_defl,caseNo);
  elseif(s==1)
   Y(ctr+1:ctr+length(tspan),:) = rk4t(@forwarddynamics2,tspan,X,ord_defl,caseNo);
  end
  
  ctr =ctr+length(tspan);  
  X =(Y(end,2:end-6))';
  tstart = tend;
  tend = tend+tstep;
  
  
  
  end     
  
  %plotData(Y,caseNo);
  figure;
  subplot(3,1,1),plot(Y(:,1),Y(:,2));
  subplot(3,1,2),plot(Y(:,1),Y(:,3));
  subplot(3,1,3),plot(Y(:,1),Y(:,4));
  
   figure;
   plot(Y(:,8),Y(:,9));
   xlabel('X');
   ylabel('Y');
   
  
      
      
