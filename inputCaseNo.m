
  function inputCaseNo(endtime)
% Input function for specifying which maneuver to perform
% CASE1:DEL_R_ORDERED=10DEGREE
% CASE2:DEL_R_ORDERED=15DEGREE
% CASE3:ZIG-ZAG MOTION: DEL_R_ORDERED=10DEGREE,WHEN PSI=10DEGREE
%        DEL_R_ORDERED=-10DEGREE
% CASE4:SAME AS CASE 3 WITH DEL_R_ORDERED=-10DEGREE INTIALLY
% CASE5:AKA CASE3:DEL_R_ORDERED=20DEGREE
% CASE6:AKA CASE4_2:DEL_R_ORDERED=-20DEGREE
% CASE7:zig zag pitch motion in vertical plane
% 
%
% INPUT : An integer specifying case number to be run
% OUTPUT : Trajectory of bot
% function inputCasenotoRun() 
  
close all;
 

% time for which the simultion runs
  timespan=0:.1:endtime;


  
% OPENING FILE CONTAINING BOT COORDINATES IN BODY FRAME
  fname1=['Bot_coordiantes','.csv'] ;
  fid=fopen(fname1,'r') ; 
  botCoordinatesInBodyFrame = csvread(fname1);
  fclose(fid);


  
% % Code for taking input from user
%   prompt = 'input case no(input 0 to quit):';
%   x = input(prompt);
%   
  x =9;
  while(x<10)
    fname=num2str(x,'%0d') ;
    Y=euler(fname,botCoordinatesInBodyFrame,timespan);
    x = x+1;% x=input(prompt);
  end
  
  
  
  r2d = 180/pi;
  % plotting u,v,w,p,q,r, phi, theta, si
   figure(1);
   subplot(3,1,1);plot(Y(:,1),Y(:,2));xlabel('time');ylabel('u');
   subplot(3,1,2);plot(Y(:,1),Y(:,3));xlabel('time');ylabel('v');
   subplot(3,1,3);plot(Y(:,1),Y(:,4));xlabel('time');ylabel('w');
  
  figure(2);
   subplot(3,1,1);plot(Y(:,1),Y(:,5));xlabel('time');ylabel('p');
   subplot(3,1,2);plot(Y(:,1),Y(:,6));xlabel('time');ylabel('q');
   subplot(3,1,3);plot(Y(:,1),Y(:,7));xlabel('time');ylabel('r');
  
  
  figure(3);
   subplot(3,1,1);plot(Y(:,1),Y(:,12)*r2d);xlabel('time');ylabel('\theta');
   subplot(3,1,2);plot(Y(:,1),Y(:,11)*r2d);xlabel('time');ylabel('\phi');
   subplot(3,1,3);plot(Y(:,1),Y(:,13)*r2d);xlabel('time');ylabel('\psi');
  %saveas(figure(2),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_stern\const_const\euler', 'eps');
   
   figure(4);
   subplot(3,1,1);plot(Y(:,1),Y(:,8)); xlabel('time');ylabel('X');
   subplot(3,1,2);plot(Y(:,1),Y(:,9)); xlabel('time');ylabel('Y');
   subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
   %saveas(figure(3),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_stern\const_const\xyz', 'eps');
   
   
   figure(5);
   %subplot(3,1,1);
   plot(Y(:,1),Y(:,17)); xlabel('time');ylabel('X');
   
%   
%    
% % % Some required parameters
% %   reqParamArray =zeros(length(timespan),12);
% %   reqParamArray(1,1) = 0;
% %   
% %   reqParamArray(i,:) =[timespan(i),Y(12),Y(2),Y(3),Y(4),Y(5),Y(6),X(6),X(7),X(8),X(9),dX(1),dX(2),dX(3)];
% %   
% %   
% %   
%   
  
  end
