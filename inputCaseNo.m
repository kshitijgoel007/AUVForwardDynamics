
  function inputCaseNo(endtime)

% Input function for specifying which maneuver to perform
%
% CASE1: DEL_R_ORDERED=10DEGREE
% CASE2: DEL_R_ORDERED=15DEGREE
% CASE3: HORIZONTAL ZIG-ZAG MOTION: DEL_R_ORDERED=10DEGREE INITIALLY
% CASE4: HORIZONTAL ZIG-ZAG MOTION: DEL_R_ORDERED=-10DEGREE INTIALLY
% CASE5: HORIZONTAL ZIG-ZAG MOTION: DEL_R_ORDERED=20DEGREE INITIALLY
% CASE6: HORIZONTAL ZIG-ZAG MOTION: DEL_R_ORDERED=-20DEGREE INITIALLY
% CASE7: VERTICAL   ZIG-ZAG MOTION: DEL_ST_ORDERED= 10DEGREE INITIALLY
% CASE8: VERTICAL   ZIG-ZAG MOTION: DEL_ST_ORDERED=-10DEGREE INITIALLY
%
% 
% INPUT : An integer specifying case number to be run and timespan 
%         
% OUTPUT: Trajectory of bot
% 
% function inputCasenotoRun() 
  
  close all;
  addpath('actuator dynamics');
  addpath('utils');  
  addpath('PDcontrol');
  addpath('excel data');

  timespan=0:.1:endtime;

  
  
  
% Initialise state vector X here
  
  X=zeros(18,1);
% u=1m/s  
  X(1,1)=1;
%  X(2,1) = -.1;



% Code for taking input from user
  prompt = 'input case no(input 0 to quit):';
  temp = input(prompt);
  caseNo=num2str(temp,'%0d') ;
  prompt2 = 'euler(0) or rk4(1)?  : ';
  sCheck = input(prompt2);
  if sCheck==0
      Y=euler(caseNo,timespan,X);
  end
  if sCheck==1
      Y=Runge_Kutta_solver(caseNo,timespan,X);
  end
  %Y=Runge_Kutta_solver(caseNo,timespan,X);
% Function for plotting
  plotData(Y, timespan);

% Function for bot visualisation
  visualiseBot(timespan,Y,caseNo);
    

  end
 
  

 %************************************************************************ 
  
   function visualiseBot(timespan,Y,caseNo)
   
% OPENING FILE CONTAINING BOT COORDINATES IN BODY FRAME
  fname1=['Bot_coordiantes','.csv'] ;
  fid=fopen(fname1,'r') ; 
  botCoordinatesInBodyFrame = csvread(fname1);
  fclose(fid);

  for i= 1:timespan(end)
        if rem(timespan(i),5)==0|| i==1   
           plotBotInGlobalFrame(caseNo,Y(11),Y(12),Y(13),Y(8),Y(9),Y(10),botCoordinatesInBodyFrame);
        end
  end  
end
  
 
%*************************************************************************
  function plotData(Y,timespan)
    
  r2d = 180/pi;
 
   figure(1);
   subplot(3,1,1);plot(timespan,Y(:,2));xlabel('time');ylabel('u');
   subplot(3,1,2);plot(timespan,Y(:,3));xlabel('time');ylabel('v');
   subplot(3,1,3);plot(timespan,Y(:,4));xlabel('time');ylabel('w');
  
  figure(2);
   subplot(3,1,1);plot(timespan,Y(:,5));xlabel('time');ylabel('p');
   subplot(3,1,2);plot(timespan,Y(:,6));xlabel('time');ylabel('q');
   subplot(3,1,3);plot(timespan,Y(:,7));xlabel('time');ylabel('r');
  
  
  figure(3);
   subplot(3,1,1);plot(timespan,Y(:,12)*r2d);xlabel('time');ylabel('\theta');
   subplot(3,1,2);plot(timespan,Y(:,11)*r2d);xlabel('time');ylabel('\phi');
   subplot(3,1,3);plot(timespan,Y(:,13)*r2d);xlabel('time');ylabel('\psi');
   %saveas(figure(2),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_stern\const_const\euler', 'eps');
   
   figure(4);
   subplot(3,1,1);plot(timespan,Y(:,8)); xlabel('time');ylabel('X');
   subplot(3,1,2);plot(timespan,Y(:,9)); xlabel('time');ylabel('Y');
   subplot(3,1,3);plot(timespan,Y(:,10)); xlabel('time');ylabel('Z');
   %saveas(figure(3),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_stern\const_const\xyz', 'eps');
   
  end