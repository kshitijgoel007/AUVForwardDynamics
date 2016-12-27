function dX = swayYawControl(t,X)

% function containing the linearised sway and yaw equations
% Purpose: PD controller implementation for course keeping 
% assumption: p = q = w = 0 ; neglect 2nd order terms

% mat file containing A and B matrices
  A_and_B_matrices;
  
      
 % take N = .5* maximum turning moment in zig-zag maneuver
   N = -.5;
   T = 1;
   F_ext=[ 0; 
           0;
          % N*cos(2*pi*t/T);
           0;
           0];  
   
   dX = A\( B*X + F_ext);
   disp(t);


end







