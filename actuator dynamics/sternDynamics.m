function del_s_dot = sternDynamics(Del_s, del_st)

% function for moving the stern by the desired ordered deflection
% calling function: forwarddynamics2

control_surf_param;
global max_st_rate;
global maxst;
dt = .1;

% Checking for the stern movement resolution
if(abs(del_st - Del_s) > 10^-5 )
     
   % Checking for the maximum stern deflection rate
     actual_del_rate_s = abs(del_st - Del_s) /dt  ;
     
     if(actual_del_rate_s >= max_st_rate)
        del_s_dot = max_st_rate;
     else    
        del_s_dot = actual_del_rate_s;
     end
     % Checked and assigned the maximum stern deflection rate

     % Assigning the direction of rotation 
     del_s_dot = del_s_dot*abs(del_st - Del_s) / (del_st - Del_s);
     % Assigned the direction of rotation

     %Assigning the stern angle limit 
     if (Del_s >= maxst) 
         del_s_dot = 0;
     elseif (Del_s <= (-maxst))
         del_s_dot = 0;
     end    
     % Assigned the stern angle limit

 elseif(abs(del_st - Del_s) <= 10^-5)    
        del_s_dot = 0;
end
end

