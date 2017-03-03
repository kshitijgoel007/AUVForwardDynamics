function del_r_dot = rudderdynamics(Del_r, del_o)

% function for moving the rudder by the desired ordered deflection
% calling function: forwarddynamics2

control_surf_param;
global max_rudd_rate;
global maxrudd;
dt = .1;

% Checking for the rudder movement resolution
  if(abs(del_o - Del_r) > 10^-5 )
     
     %checking for the maximum rudder deflection rate
     actual_del_rate_s = abs(del_o - Del_r) /dt  ;
     
     if(actual_del_rate_s >= max_rudd_rate)
        del_r_dot = max_rudd_rate;
     else    
        del_r_dot = actual_del_rate_s;
     end
     % Checked and assigned  rudder rate

     % Assigning the direction of rotation 
     del_r_dot = del_r_dot*abs(del_o - Del_r) / (del_o - Del_r);
     % Assigned the direction of rotation

     %Assigning the rudder angle limit 
     if (Del_r >= maxrudd) 
         del_r_dot = 0;
     elseif (Del_r <= (-maxrudd))
         del_r_dot = 0;
     end    
     % Assigned the rudder angle limit

  elseif(abs(del_o - Del_r) <= 10^-5)    
        del_r_dot = 0;
  end
  
  
end


