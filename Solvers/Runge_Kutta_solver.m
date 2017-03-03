
  function Y=Runge_Kutta_solver(caseNo,timespan,X)

% Function for solving 6DOF equations using 1st order euler method
%
% INPUT : caseno :                    index no of the type of maneuver to be performed
%         botCoordinatesInbodyFrame : 3185X3 matrix containing (x,y,z)
%                                     coordinates of the bot in body frame
%         timespan :                  simulation run-time
%         X:                          initial state(t = 0)

% OUTPUT : plots of Trajectory of bot
%          CSV file containing (X,Y,Z) of bot at different time instants 
%          CSV file containing values of state vector X at different time
%          instants
%          Output vector Y
%
% function Y=euler(caseNo,botCoordinatesInBodyFrame,timespan)

addpath('actuator dynamics');
addpath('utils');
addpath('PDcontrol');


% timstep
  dt=.1;
% Mat file containing properties of bot
  geoprop;                       


% Initialising output vector for storing values of state vector against time
% for different time instants
  Y=zeros(length(timespan),length(X)+1);
% First column stores time
  Y(1,1)=0;                      
% INITIAL VALUE SAVED IN Y
  for i=1:length(X)
    Y(1,i+1)=X(i);            
  end
  
  
  
% Initialising ordered control surface deflections array
  ord_defl=zeros(4,1);
% orderedDeflectionsArray(1)=del_ordered_rudder
% orderedDeflectionsArray(2)=del_ordered_stern
% orderedDeflectionsArray(3)=del_ordered_bp
% orderedDeflectionsArray(1)=del_ordered_bs


%**************************************************************************  
  
 



switch caseNo
       
      case {'1'}
      % when del_o is positive, bot turns to port(-Y initially points to port
      % also si is -ve.
      ord_defl(1)=10*pi/180.0;
      Y=constantTurn(caseNo,X,Y,timespan,dt,ord_defl);
        
%       figure(1);
%       subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
%       subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
%       subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
%       %hold on;subplot(3,1,1);plot(Y(:,1),ord_defl(1)*180/pi);
%       title('Turning manoeuvre with ordered rudder deflection = 10 degree');
%       %saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseI', 'eps');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Turning manoeuvre with ordered rudder deflection = 10 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Turning manoeuvre with ordered rudder deflection = 10 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Turning manoeuvre with ordered rudder deflection = 10 degree');
%       
      %saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xy', 'eps');
      %saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\zt', 'eps');
      %saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_r', 'eps');
      
    
      case {'2'}
      ord_defl(1)=15*pi/180.0;
      Y=constantTurn(caseNo,X,Y,timespan,dt,ord_defl);
   
%       figure(1);
%       subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
%       subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
%       subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
%       %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi,'-or');
%       title('Turning manoeuvre with ordered rudder deflection = 15 degree');
%       saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseII', 'eps');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Turning manoeuvre with ordered rudder deflection = 15 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Turning manoeuvre with ordered rudder deflection = 15 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Turning manoeuvre with ordered rudder deflection = 15 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase2', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase2', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase2', 'eps');
      
   
      case {'3'}
      ord_defl(1)=-10*pi/180.0;
      Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,ord_defl);
      
%       figure(1);
%       subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
%       subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
%       subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
%       %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
%       title('Horizontal zig-zag with ordered rudder deflection = 10 degree');
%       saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIII', 'eps');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Horizontal zig-zag with ordered rudder deflection = 10 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Horizontal zig-zag with ordered rudder deflection = 10 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Horizontal zig-zag with ordered rudder deflection = 10 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase3', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase3', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase3', 'eps');
%       
      
      case {'4'}
      ord_defl(1)=10*pi/180.0;
      Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,ord_defl);
    
%       figure(1);
%       subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
%       subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
%       subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
%       %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
%       title('Horizontal zig-zag with ordered rudder deflection =  -10 degree');
%       saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIV', 'eps');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Horizontal zig-zag with ordered rudder deflection = -10 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Horizontal zig-zag with ordered rudder deflection = -10 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Horizontal zig-zag with ordered rudder deflection = -10 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase4', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase4', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase4', 'eps');

      
      case {'5'}
      ord_defl(1)=-20*pi/180.0;
      Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,ord_defl);

%       figure(1);
%       subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
%       subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
%       subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
%       %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
%       title('Horizontal zig-zag with ordered rudder deflection = 20 degree');
%       saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseV', 'eps');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Horizontal zig-zag with ordered rudder deflection = 20 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Horizontal zig-zag with ordered rudder deflection = 20 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Horizontal zig-zag with ordered rudder deflection = 20 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase5', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase5', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase5', 'eps');
%      
    
      case {'6'}
      ord_defl(1)=20*pi/180.0;
      Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,ord_defl);
      
%       figure(1);
%       subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
%       subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
%       subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
%       %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
%       title('Horizontal zig-zag with ordered rudder deflection = -20 degree');
%       saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseVI', 'eps');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Horizontal zig-zag with ordered rudder deflection = -20 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Horizontal zig-zag with ordered rudder deflection = -20 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Horizontal zig-zag with ordered rudder deflection = -20 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase6', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase6', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase6', 'eps');
%      
     
      case {'7'}
      ord_defl(2)=-10*pi/180.0;
      Y=zigzagInVerticalPlane(caseNo,X,Y,timespan,dt,ord_defl);
      
%       figure(1);
%       subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
%       subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
%       subplot(3,1,1);plot(Y(:,1),Y(:,14)*180/pi); xlabel('time');ylabel('\delta_st');
%       %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
%       title('vertical zig-zag with ordered stern deflection = 10 degree');
%       saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseVII', 'eps');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('vertical zig-zag with ordered stern deflection = 10 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('vertical zig-zag with ordered stern deflection = 10 degree');
%       figure(4);plot(Y(:,1),Y(:,14)*180/pi); xlabel('time');ylabel('\delta_st');title('vertical zig-zag with ordered stern deflection = 10 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase7', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase7', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase7', 'eps');
%       
      
      case {'8'}
      ord_defl(2)=10*pi/180.0;
      Y=zigzagInVerticalPlane(caseNo,X,Y,timespan,dt,ord_defl);
    
%       figure(1);
%       subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
%       subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
%       subplot(3,1,1);plot(Y(:,1),Y(:,14)*180/pi); xlabel('time');ylabel('\delta_s');
%       %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
%       title('Vertical zig-zag with ordered rudder deflection =  -10 degree');
%       saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseVIII', 'eps');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Vertical zig-zag with ordered rudder deflection = -10 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Vertical zig-zag with ordered rudder deflection = -10 degree');
%       figure(4);plot(Y(:,1),Y(:,14)*180/pi); xlabel('time');ylabel('\delta_st');title('Vertical zig-zag with ordered rudder deflection = -10 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase8', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase8', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase8', 'eps');

      
      case { '9'}
      
      Y=constantTurn(caseNo,X,Y,timespan,dt,ord_defl);
        
end % end of switch


% SAVING Y VECTOR
  fname1=['case_',caseNo,'.csv'] ;
  fid=fopen(fname1,'w') ;
  dlmwrite(fname1,Y);
  fclose(fid);
end
  
%*************************************************************************
%*************************************************************************


function Y=constantTurn(caseNo,X,Y,timespan,dt,ord_defl)
% Function implementing turn in horizontal plane with constant rudder deflection
Y=rk4t(@forwarddynamics2,timespan,X,ord_defl,caseNo);
 end
%*************************************************************************
%*************************************************************************

 function Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,ord_defl)
 
 Y=rk4t(@forwarddynamics2,timespan,X,ord_defl,caseNo);
      % NOTE : ruddder deflection to starboard side is -ve
      %        X(12)(yaw) towards starboard side is +ve      
 end
 
%*******************************************************************
 
 function Y=zigzagInVerticalPlane(caseNo,X,Y,timespan,dt,ord_defl)
  % NOTE : ruddder deflection to starboard side is -ve
  %        X(12)(yaw) towards starboard side is +ve
               
     Y=rk4t(@forwarddynamics2,timespan,X,ord_defl,caseNo);
 %end
 
 
 end
 