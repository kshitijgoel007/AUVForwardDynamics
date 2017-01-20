function [ Y ] = getSensorData(X, U, vel_bf, wb )
%GETSENSORDATA outputs DVL and pressure sensor data.

global DVL_to_body;
global dvl_location;
global r_cg;

d = [(dvl_location(1) -  r_cg(1));
     (dvl_location(2) -  r_cg(2));
     (dvl_location(3) -  r_cg(3))];
 
v = DVL_to_body*dvl_model(vel_bf, wb)' - cross( U(4:6) ,d ) ;

Y = v;

end

