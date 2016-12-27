function del_bp_dot = bowPortDynamics(Del_bp, del_bp)

% function for moving the bow by the desired ordered deflection
% calling function: forwarddynamics2

addpath('utils');  
control_surf_param;
global max_bp_rate;
global maxbp;
dt = .1;
if(abs(del_bp - Del_bp) > 10^-5 )
     
     %Checking for the maximum bowplane deflection rate
     actual_del_rate_s = abs(del_bp - Del_bp) /dt  ;
     
     if(actual_del_rate_s >= max_bp_rate)
        del_bp_dot = max_bp_rate;
     else    
        del_bp_dot = actual_del_rate_s;
     end
     %Checked and assigned the maximum stern rate

     %Assigning the direction of rotation 
     del_bp_dot = del_bp_dot*abs(del_bp - Del_bp) / (del_bp - Del_bp);
     %Assigned the direction of rotation

     %Assigning the bowplane angle limit 
     if (Del_bp >= maxbp) 
         del_bp_dot = 0;
     elseif (Del_bp <= (-maxbp))
              del_bp_dot = 0;
     end    
     %Assigned the bowplane angle limit

 elseif(abs(del_bp - Del_bp) <= 10^-5)    
        del_bp_dot = 0;
end
