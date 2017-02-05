
  function runSim()

% Input function for specifying which maneuver to perform

% u =1,1.5,2m/s
% CASE1: TURNING DEL_R_ORDERED=10DEGREE
% CASE2: TURNING DEL_R_ORDERED=15DEGREE
% CASE3: HORIZONTAL ZIG-ZAG MOTION: DEL_R_ORDERED=10DEGREE INITIALLY
% CASE4: HORIZONTAL ZIG-ZAG MOTION: DEL_R_ORDERED=-10DEGREE INTIALLY
% CASE5: HORIZONTAL ZIG-ZAG MOTION: DEL_R_ORDERED=20DEGREE INITIALLY
% CASE6: HORIZONTAL ZIG-ZAG MOTION: DEL_R_ORDERED=-20DEGREE INITIALLY
% CASE7: VERTICAL   ZIG-ZAG MOTION: DEL_ST_ORDERED= 10DEGREE INITIALLY
% CASE8: VERTICAL   ZIG-ZAG MOTION: DEL_ST_ORDERED=-10DEGREE INITIALLY
% CASE9: STEERING PD CONTROLLER
% CASE10:  DIVING PD CONTROLLER
% CASE 11: TURNING DEL_R_ORDERED = -10 DEGREE, 
% CASE 12: TURNINGDEL_R_ORDERED = -15 DEGREE, 
% CASE 13: VERTICAL ZIG-ZAG MOTION: DEL_ST_ORDERED = 20DEGREE INITIALLY,
% CASE 14: VERTICAL ZIG-ZAG MOTION: DEL_ST_ORDERED = -20 DEGREE INITIALLY

% INPUT : An integer specifying case number to be run and timespan 
%         
% OUTPUT: Trajectory of bot
% 
% function runsim() 


  clc;
  close all;
  addpath('actuator dynamics');
  addpath('utils');  
  addpath('PDcontrol');
  addpath('Solvers');
  addpath('excel data');
  addpath('excel data/u_1m_s');
  addpath('excel data/u_1dot5m_s');
  addpath('excel data/u_2m_s');
  
  r2d =180/pi;
  lbl = {'t(s)', 'u(m/s)','v(m/s)','w(m/s)',...
         'p(deg/s)','q(deg/s)','r(deg/s)'...
          'X(m)','Y(m)','Z(m)','phi(deg)','theta(deg)','psi(deg)'...
          'Del_s(deg)','Del_bp(deg)','Del_bs(deg)','Del_r(deg)',...
          'Del_delb',...
          'n(RPS)','ax(m/s2)','ay(m/s2)','az(m/s2)',...
          'alphax(deg/s2)','alphay(deg/s2)','alphaz(deg/s2)'};
          
  

%% ************* Code for taking input from user**********************
 
%Initialise state vector X here
  
X = init_state();


prompt = 'input case no(input 0 to quit):';
temp = input(prompt);
  
  
if(temp~=0)
     
     caseNo=num2str(temp,'%0d') ;
     prompt = 'input simulation time(sec):';
     endtime = input(prompt);timespan=0:.1:endtime;
     
     prompt = 'euler(0) or rk4(1)?  : ';
     sCheck = input(prompt);
     Y=callSolver(caseNo,timespan,X,sCheck);
     plotData(Y,caseNo);
     
     % SAVING Y VECTOR
     % angle conversion to degrees
     temp = Y;
     temp(:,5:7) = temp(:,5:7)*r2d;
     temp(:,11:17)= temp(:,11:17)*r2d;
     temp(:,23:25) = temp(:,23:25)*r2d;
     fname1=['excel data/case_',caseNo,'.csv'] ;
     fid=fopen(fname1,'a') ;
     dlmwrite(fname1,temp,'-append','precision',4);
     fclose(fid);


    % Function for bot visualisation
    %  visualiseBot(timespan,Y,caseNo);

end


%% ***************** code for getting 30 min of data at 100 hz*******************
% tstart =0;
% tstep =.01;
% cstep=225;
% tend = cstep;
% 
% %initialising state vector
% X = zeros(18,1);
% X(18) = 110.969;
% X(1) = 1;
% 
% 
% %setting solver to euler
% sCheck =0;
% 
% 
% %running simulation
% 
% %writing labels
% fid  = fopen('excel data/30minSimData.csv','w');
%       [rows,cols]=size(lbl);
%        for k=1:rows
%            fprintf(fid,'%s,',lbl{k,1:end-1});
%            fprintf(fid,'%s\n',lbl{k,end});
%        end
% fclose(fid);
% 
% 
% for caseNo = 1:8
%     tspan = tstart:tstep:tend;
%   
%     Y=callSolver(caseNo,tspan,X,sCheck);
%     
%       
%       %saving data
%       fid  = fopen('excel data/30minSimData.csv','a');
%       dlmwrite('excel data/30minSimData.csv',Y,'-append');
%       fclose(fid) ;
% 
%       %updating intial state and tspan for next case a
%       X(:,1) = Y(end,2:end-6);
%       tstart = tend+tstep;
%       tend = tend+cstep;
% end



   display('simulation done');
  end
 
  
  
  

 %% %************************************************************************ 
  
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
 
  
  