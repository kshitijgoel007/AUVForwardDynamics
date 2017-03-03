function del_bs_dot = bowStarboardDynamics(Del_bs, del_bs)

% function for moving the bow by the desired ordered deflection
% calling function: forwarddynamics2

control_surf_param;
global max_bs_rate;
global maxbs;
dt = .1;

if(abs(del_bs - Del_bs) > 10^-5 )
     
     %Checking for the maximum bowplane deflection rate
     actual_del_rate_s = abs(del_bs - Del_bs) /dt  ;
     
     if(actual_del_rate_s >= max_bs_rate)
        del_bs_dot = max_bs_rate;
     else    
        del_bs_dot = actual_del_rate_s;
     end
     %Checked and assigned the maximum stern rate

     %Assigning the direction of rotation 
     del_bs_dot = del_bs_dot*abs(del_bs - Del_bs) / (del_bs - Del_bs);
     %Assigned the direction of rotation

     %Assigning the bowplane angle limit 
     if (Del_bs >= maxbs) 
         del_bs_dot = 0;
     elseif (Del_bs <= (-maxbs))
              del_bs_dot = 0;
     end    
     %Assigned the bowplane angle limit

 elseif(abs(del_bs - Del_bs) <= 10^-5)    
        del_bs_dot = 0;
end
end
