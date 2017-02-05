  function del_r_dot =rudderDynamicsForSwayYawcontrol(Del_r,psi,r)

% MODELLING RUDDER DEFLECTION FOR TRACKING CONTROL USING PD CONTROLLER
% Calling fn : forwarddynamics2 
  addpath('utils');  
  control_surf_param;
  geoprop;
  
  global maxrudd;
  global max_rudd_rate;
  
  U = 1;
  TE =2.5;%sec
  a = 1;
  b = 1.5*L/U;
  
  
  if abs(Del_r)>maxrudd
      Del_r = maxrudd*(Del_r/abs(Del_r));
  end
  
  del_r_dot = (-Del_r +a*psi +b*r)/TE;
 
  if(abs(del_r_dot)>= max_rudd_rate)
   del_r_dot = max_rudd_rate;
   
  end
