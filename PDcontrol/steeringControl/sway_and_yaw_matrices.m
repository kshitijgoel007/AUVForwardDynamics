  geoprop;
  yawderivatives;
  swayderivatives;

  
  syms k1 k2 k3 k4 ;
% k1, k2, k3, k4 : for nondimensionalising;
% a = Kp, b = kd
% U = x velocity, assumed 1 for now
% TE = time lag of actuator, assumed 2.5 sec
  
  k1 = rho*(L^3)/2;
  k2 = rho*(L^5)/2;
  U  = 1;
  TE = 2.5;
  k3 = U/L;
  k4 =1/k3;
  m = W/9.81;
  a = 0; b= 0;
  

   A = [(m/k1)-sway_deriv(5)          0              0            0 ;
          0                 (Iz/k2)-yaw_deriv(2)     0            0 ;
          0                         0            TE/k4         -b/k4 ;
          0                         0              0            1 ];
  
   
   B = [sway_deriv(11)      sway_deriv(7)-(m/k1)  sway_deriv(13)  0 ;
        yaw_deriv(11)          yaw_deriv(7)      yaw_deriv(13)  0 ;
          0                         0            -1          a;
          0                         1               0           0];


      
% storing the a and b values corresponding to -ve eigenvalues of B 
  
a_b_valuesArray = check_eigen(A,B,k4);

fname2=['a_b_array_swayYaw','.csv'] ;
fid2=fopen(fname2,'w');
dlmwrite(fname2,a_b_valuesArray);
fclose(fid2) ;

    