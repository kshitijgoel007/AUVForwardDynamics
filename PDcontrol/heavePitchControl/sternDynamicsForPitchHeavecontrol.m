  function del_s_dot =sternDynamicsForPitchHeavecontrol(Del_s,theta,q)

% MODELLING RUDDER DEFLECTION FOR TRACKING CONTROL USING PD CONTROLLER
% Calling fn : forwarddynamics2 
  addpath('utils');  
  control_surf_param;
  geoprop;
  
  global maxst;
  global max_st_rate;
  
  
  TE =2.5;%sec
  a = .9;
  b = 8;
  
  
  if abs(Del_s)>maxst
      Del_s = maxst*(Del_s/abs(Del_s));
  end
  
  del_s_dot = (-Del_s +a*theta +b*q)/TE;
 
  if(abs(del_s_dot)>= max_st_rate)
   del_s_dot = max_st_rate*(del_s_dot/abs(del_s_dot));
   
  end
