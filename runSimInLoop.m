function runSimInLoop()

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
% CASE 12: TURNING DEL_R_ORDERED = -15 DEGREE, 
% CASE 13: VERTICAL ZIG-ZAG MOTION: DEL_ST_ORDERED = 20DEGREE INITIALLY,
% CASE 14: VERTICAL ZIG-ZAG MOTION: DEL_ST_ORDERED = -20 DEGREE INITIALLY

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
 
%setting solver to euler
sCheck =0;

% setting time span
tspan = 0:.1:1;

  
% code for running additional cases

% ...................%u = 1m/s.............................................
X = zeros(18,1);
X(18) = 110.969;
X(1) = 1;



%running simulation
for i = 10:10
   caseNo =num2str(i); 
    Y=callSolver(caseNo,tspan,X,sCheck);
    
    %SAVING Y VECTOR
    %angle conversion to degrees
     temp = Y;
     temp(:,5:7) = temp(:,5:7)*r2d;
     temp(:,11:17)= temp(:,11:17)*r2d;
     temp(:,23:25) = temp(:,23:25)*r2d;
     
     
     %writing labels
     fname1=['excel data/u_1m_s/case_',caseNo,'.csv'] ;
     fid  = fopen(fname1,'w');
     [rows,cols]=size(lbl);
      for k=1:rows
           fprintf(fid,'%s,',lbl{k,1:end-1});
           fprintf(fid,'%s\n',lbl{k,end});
      end
      fclose(fid);

     fid=fopen(fname1,'a') ;
     dlmwrite(fname1,temp,'-append','precision',4);
     fclose(fid);

end
end
% %............................u = 1.5m/s................................
% X = zeros(18,1);
% X(18) = 163;
% X(1) = 1.5;
% 
% 
% 
% %running simulation
% for i = 1:14
%    caseNo =num2str(i); 
%     Y=callSolver(caseNo,tspan,X,sCheck);
%     
%     %SAVING Y VECTOR
%     %angle conversion to degrees
%      temp = Y;
%      temp(:,5:7) = temp(:,5:7)*r2d;
%      temp(:,11:17)= temp(:,11:17)*r2d;
%      temp(:,23:25) = temp(:,23:25)*r2d;
%      
%      
%      %writing labels
%      fname1=['excel data/u_1dot5m_s/case_',caseNo,'.csv'] ;
%      fid  = fopen(fname1,'w');
%      [rows,cols]=size(lbl);
%       for k=1:rows
%            fprintf(fid,'%s,',lbl{k,1:end-1});
%            fprintf(fid,'%s\n',lbl{k,end});
%       end
%       fclose(fid);
% 
%      fid=fopen(fname1,'a') ;
%      dlmwrite(fname1,temp,'-append','precision',4);
%      fclose(fid);
% 
% end
% 
% ........................u = 2m/s..............................
% X = zeros(18,1);
% X(18) = 215.3;
% X(1) = 2;
% 
% 
% 
% 
% %running simulation
% for i = 1:14
%     caseNo =num2str(i); 
%     Y=callSolver(caseNo,tspan,X,sCheck);
%     
%     %SAVING Y VECTOR
%     %angle conversion to degrees
%      temp = Y;
%      temp(:,5:7) = temp(:,5:7)*r2d;
%      temp(:,11:17)= temp(:,11:17)*r2d;
%      temp(:,23:25) = temp(:,23:25)*r2d;
%      
%      
%      %writing labels
%      fname1=['excel data/u_2m_s/case_',caseNo,'.csv'] ;
%      fid  = fopen(fname1,'w');
%      [rows,cols]=size(lbl);
%       for k=1:rows
%            fprintf(fid,'%s,',lbl{k,1:end-1});
%            fprintf(fid,'%s\n',lbl{k,end});
%       end
%       fclose(fid);
% 
%      fid=fopen(fname1,'a') ;
%      dlmwrite(fname1,temp,'-append','precision',4);
%      fclose(fid);
% 
% end
% 
