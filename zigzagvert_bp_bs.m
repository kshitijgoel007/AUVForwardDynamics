%script for testing zig-zag
function zigzagvert_bp_bs(tend)
close all;
t=0:.1:tend;
dt=.1;
global Y del_bp del_bs
%MAT FILE CONTAING PROPERTIES OF BOT
geoprop;                       
r2d = 180/pi;
X=zeros(18,1);
X(1)=1; % initial surge velocity
Y=zeros(length(t),length(X)+1);
%first column stores time
Y(1,1)=0;                      

%INITIAL VALUES SAVED IN Y
for i=1:length(X)
    Y(1,i+1)=X(i);            
end
%disp(del_o);
%implementing zig-zag
%global del_bp;
%global del_bs;
%global del_s;
del_bp=+10*pi/180.0;
del_bs=+10*pi/180.0;
%del_s =+10*pi/180.0;
   for i=2:length(t)
    %SOLVING USING EULER METHOD
    DX=forwarddynamics2(t(i),X);
    temp=X+DX*dt;
    X=temp;
    Y(i,1)=t(i);
    for j=1:length(X)
    Y(i,j+1)=X(j);
    end
    
    %EULER METHOD IMPLEMENTED 
    %bot will turn opposite to control surface deflection
    if (X(11)>10*pi/180.0)
      del_bp= 0*(pi/180);
      del_bs= 0*pi/180;
       %del_s=10*pi/180.0;
    else if (X(11)<-10*pi/180.0)
      del_bp=0*pi/180.0;
      del_bs=0*pi/180.0;
%       % del_s=-10*pi/180.0;
        end
    end
   %plot(t,Y(:,11));
   end
   
   figure(1);
   subplot(3,1,1);plot(t,Y(:,12)*r2d);xlabel('time');ylabel('\theta (in degrees)');
   subplot(3,1,2);plot(t,Y(:,11)*r2d);xlabel('time');ylabel('\phi (in degrees)');
   subplot(3,1,3);plot(t,Y(:,13)*r2d);xlabel('time');ylabel('\psi (in degrees)');
  saveas(figure(1),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_bp_bs\const_zero\euler.fig');
   
   figure(2);
   subplot(3,1,1);plot(t,Y(:,8)); xlabel('time');ylabel('X');
   subplot(3,1,2);plot(t,Y(:,9)); xlabel('time');ylabel('Y');
   subplot(3,1,3);plot(t,Y(:,10)); xlabel('time');ylabel('Z');
   saveas(figure(2),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_bp_bs\const_zero\xyz.fig');
   
   figure(3);
   subplot(2,1,1);plot(t, Y(:,15));xlabel('time');ylabel('\delta_{b_{s,actual}}');
   subplot(2,1,2);plot(t, Y(:,16));xlabel('time');ylabel('\delta_{b_{p,actual}}');
   saveas(figure(3),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_bp_bs\const_zero\delbsbp.fig');
end