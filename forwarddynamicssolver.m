clear all;
clc;
tspan = [0 600];

%forwarddyanmics2 returns dX which is the differential equation to be solved to get X
%ODE45 solves dX and returns 2 vectors which get stored in t and X,which
%are then again passed to forwarddynamics2
%T is a column vector,each row of y corresponds tothe corresponding row of
%T.
%the first column of each row of y gives u for the corresponding time instant,2nd column give v, and so on 
[T,y] = ode45( @forwarddynamics2, tspan, [1; 0; 0; 0; 0; 0;
                                          0; 0; 0; 0; 0; 0;
                                          0; 0; 0; 0; 0; 0]);
%alternatively[T,y] = ode45( @(t,x)forwarddynamics2(t,x), tspan, [0.5 0 0 0 0 0]);
disp(y(:,1));%displays the 1st column of y ,i.e. the values of u for 0-1000 sec
%plot(T,y(:,1));
