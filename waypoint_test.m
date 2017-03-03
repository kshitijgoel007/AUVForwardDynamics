
close all;
clc;
addpath('utils');
addpath('PDcontrol/heavePitchControl');
addpath('PDcontrol/swayYawControl');
addpath('Solvers');
 
X_initial_s = [0; 0; 0; 0];%v,r,del_r,psi
X_initial_h = [0; 0; 0; 0];%w,q,del_s,theta


  
%   ansx =1;
%   s=0;
%   
       
  prompt = 'input x-coordinate:';
  ansx = input(prompt);
  prompt = 'input y-coordinate:';
  ansy = input(prompt);
  prompt = 'input z-coordinate:';
  ansz = input(prompt);
  prompt = 'solver?? euler(0)/RK4(1)';
  s = input(prompt);
  
  yaw_d= atan(ansy/ansx);
  pitch_d = atan(ansz/sqrt(ansx*ansx + ansy*ansy));
  caseNo = '11';
  
  tstart =0;
  tstep = 1;
  tend = tstep;
 
  % simulation running time
  runtime = 15 ;
  
  %frequency
  dt = .1;
  
  ord_defl = [0;0;0;0];
  
  X = init_state();% initialise everything to zero

  ctr =0;
  
  % LOGIC USED:
  % run all controllers for tstep sec, then check if goal is reached
  % and vary control input accordingly
  
  while( tend < runtime)
      
  % running PD control for tstep sec 
   tspan = tstart:dt:tend;
   
  if(s==0)
  
  Y(ctr+1:ctr+length(tspan),:) = euler(@forwarddynamics2,tspan,X, ord_defl,caseNo);
  
  elseif(s==1)
      
   Y(ctr+1:ctr+length(tspan),:) = rk4t(@forwarddynamics2,tspan,X,ord_defl,caseNo);
   
  end
  
  
  
  
  ctr =ctr+length(tspan);  
  X =(Y(end,2:end))';
  tstart = tend;
  tend = tend+tstep;
  
  % if goal reached, stop thrusters
  if(abs(X(7)-ansx)<.1), X(18)=-110.657; end
  
  disp(X(7));
  end     
  
  plotData(Y,caseNo);
  figure;
  subplot(3,1,1),plot(Y(:,1),Y(:,2));
  subplot(3,1,2),plot(Y(:,1),Y(:,3));
  subplot(3,1,3),plot(Y(:,1),Y(:,4));
  
   figure;
   plot(Y(:,8),Y(:,9));
   
  
      
      
