 function plotData(Y,caseNo)
    r2d = 180/pi;
  
%  
   figure(1);
   subplot(3,1,1);plot(Y(:,1),Y(:,2));xlabel('time');ylabel('u');
   subplot(3,1,2);plot(Y(:,1),Y(:,3));xlabel('time');ylabel('v');
   subplot(3,1,3);plot(Y(:,1),Y(:,4));xlabel('time');ylabel('w');
  

%   figure(2);
%    subplot(3,1,1);plot(Y(:,1),Y(:,5));xlabel('time');ylabel('p');
%    subplot(3,1,2);plot(Y(:,1),Y(:,6));xlabel('time');ylabel('q');
%    subplot(3,1,3);plot(Y(:,1),Y(:,7));xlabel('time');ylabel('r');
%   
%   
%   figure(3);
%    subplot(3,1,1);plot(Y(:,1),Y(:,12)*r2d);xlabel('time');ylabel('\theta');
%    subplot(3,1,2);plot(Y(:,1),Y(:,11)*r2d);xlabel('time');ylabel('\phi');
%    subplot(3,1,3);plot(Y(:,1),Y(:,13)*r2d);xlabel('time');ylabel('\psi');
%    
   figure(4);
   subplot(3,1,1);plot(Y(:,1),Y(:,8)); xlabel('time');ylabel('X');
   subplot(3,1,2);plot(Y(:,1),Y(:,9)); xlabel('time');ylabel('Y');
   subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
%    %saveas(figure(3),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_stern\const_const\xyz', 'eps');
%    
%    
%    figure(5);
%    subplot(3,1,1);
%    plot(Y(:,1),Y(:,17)); xlabel('time');ylabel('\delta_r');
%    
   
   
   
switch caseNo
       
      case {'1'}
      
      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
      subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
      subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');

      title('Turning maneuver with ordered rudder deflection = 10 degree');
     % saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseI', 'eps');
     % saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseI', 'jpg');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Turning manoeuvre with ordered rudder deflection = 10 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Turning manoeuvre with ordered rudder deflection = 10 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Turning manoeuvre with ordered rudder deflection = 10 degree');
%       
      %saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xy', 'eps');
      %saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\zt', 'eps');
      %saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_r', 'eps');
      
    
      case {'2'}
      
      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
      subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
      subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi,'-or');
      title('Turning manoeuvre with ordered rudder deflection = 15 degree');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseII', 'eps');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseII', 'jpg');
%      
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Turning manoeuvre with ordered rudder deflection = 15 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Turning manoeuvre with ordered rudder deflection = 15 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Turning manoeuvre with ordered rudder deflection = 15 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase2', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase2', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase2', 'eps');
      
   
      case {'3'}
      
      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
      subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
      subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
      title('Horizontal zig-zag with ordered rudder deflection = 10 degree');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIII', 'eps');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIII', 'jpg');
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
      
      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
      subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
      subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
      title('Horizontal zig-zag with ordered rudder deflection =  -10 degree');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIV', 'eps');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIV', 'jpg');
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Horizontal zig-zag with ordered rudder deflection = -10 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Horizontal zig-zag with ordered rudder deflection = -10 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Horizontal zig-zag with ordered rudder deflection = -10 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase4', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase4', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase4', 'eps');

      
      case {'5'}
      
      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
      subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
      subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
      title('Horizontal zig-zag with ordered rudder deflection = 20 degree');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseV', 'eps');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseV', 'jpg');
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Horizontal zig-zag with ordered rudder deflection = 20 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Horizontal zig-zag with ordered rudder deflection = 20 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Horizontal zig-zag with ordered rudder deflection = 20 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase5', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase5', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase5', 'eps');
%      
    
      case {'6'}
      
      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
      subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
      subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
      title('Horizontal zig-zag with ordered rudder deflection = -20 degree');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseVI', 'eps');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseVI', 'jpg');
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Horizontal zig-zag with ordered rudder deflection = -20 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Horizontal zig-zag with ordered rudder deflection = -20 degree');
%       figure(4);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');title('Horizontal zig-zag with ordered rudder deflection = -20 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase6', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase6', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase6', 'eps');
%      
     
      case {'7'}

      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,10)); xlabel('X');ylabel('Z');
      subplot(3,1,3);plot(Y(:,1),Y(:,9)); xlabel('time');ylabel('Y');
      subplot(3,1,1);plot(Y(:,1),Y(:,14)*180/pi); xlabel('time');ylabel('\delta_st');
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
      title('vertical zig-zag with ordered stern deflection = 10 degree');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseVII', 'eps');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseVII', 'jpg');
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
      
    
      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,10)); xlabel('X');ylabel('Z');
      subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Y');
      subplot(3,1,1);plot(Y(:,1),Y(:,14)*180/pi); xlabel('time');ylabel('\delta_s');
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
      title('Vertical zig-zag with ordered stern deflection =  -10 degree');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseVIII', 'eps');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseVIII', 'jpg');
%       figure(2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');title('Vertical zig-zag with ordered rudder deflection = -10 degree');
%       figure(3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');title('Vertical zig-zag with ordered rudder deflection = -10 degree');
%       figure(4);plot(Y(:,1),Y(:,14)*180/pi); xlabel('time');ylabel('\delta_st');title('Vertical zig-zag with ordered rudder deflection = -10 degree');
%       
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\xycase8', 'eps');
%       saveas(figure(3),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\ztcase8', 'eps');
%       saveas(figure(4),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\delta_rcase8', 'eps');

      
case {'9'}
      
    
      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,9)); xlabel('X');ylabel('Y');
      subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
      subplot(3,1,1);plot(Y(:,1),Y(:,17)*180/pi); xlabel('time');ylabel('\delta_r');
      
%       figure(2)
%       subplot(2,1,1);plot(Y(:,1),D(2),'-or'); xlabel('time');ylabel('disturbance in v');
%       subplot(2,1,2);plot(Y(:,1),D(6),'-or'); xlabel('time');ylabel('disturbance in r');
%       
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
      title('sway Yaw PD control');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIX', 'eps');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIX', 'jpg');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIX(2)', 'eps');
      %%saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseIX(2)', 'jpg');

case {'10'}
      
    
      figure(1);
      subplot(3,1,2);plot(Y(:,8),Y(:,10)); xlabel('X');ylabel('Z');
      subplot(3,1,3);plot(Y(:,1),Y(:,9)); xlabel('time');ylabel('Y');
      subplot(3,1,1);plot(Y(:,1),Y(:,14)*180/pi); xlabel('time');ylabel('\delta_s');
      
%       figure(2)
%       subplot(2,1,1);plot(Y(:,1),D(3),'-or'); xlabel('time');ylabel('disturbance in w');
%       subplot(2,1,2);plot(Y(:,1),D(5),'-or'); xlabel('time');ylabel('disturbance in q');
%       
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
      title('Pitch Heave PD control');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseX', 'eps');
       saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseX', 'jpg');
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseX(2)', 'eps');
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseX(2)', 'jpg');

   
case {'11'}
      
    
      figure(1);
      subplot(3,1,1);plot(Y(:,1),Y(:,8)); xlabel('time');ylabel('X');
      subplot(3,1,2);plot(Y(:,1),Y(:,9)); xlabel('time');ylabel('Y');
      subplot(3,1,3);plot(Y(:,1),Y(:,10)); xlabel('time');ylabel('Z');
      
      figure(2);
      plot3(Y(:,8),Y(:,9),Y(:,10));
      
%       figure(2)
%       subplot(2,1,1);plot(Y(:,1),D(3),'-or'); xlabel('time');ylabel('disturbance in w');
%       subplot(2,1,2);plot(Y(:,1),D(5),'-or'); xlabel('time');ylabel('disturbance in q');
%       
      %hold on;subplot(3,1,1);plot(Y(:,1),orderedControlSurfaceDeflectionArray(1)*180/pi);
      title('waypoint');
      saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseXI', 'eps');
       saveas(figure(1),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseXI', 'jpg');
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseX(2)', 'eps');
%       saveas(figure(2),'C:\Users\Shubham\Desktop\AUV\AUV_DYNAMIC_MODELLING\AUVForwardDynamics\plots\CaseX(2)', 'jpg');




end


end
