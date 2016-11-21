%script for testing zig-zag
function zigzagvert(tend)
close all;
t=0:.1:tend;
dt=.1;
global Y;
%MAT FILE CONTAING PROPERTIES OF BOT
geoprop;                       

X=zeros(18,1);
X(1)=1;
Y=zeros(length(t),length(X)+1);
%first column stores time
Y(1,1)=0;                      

%INITIAL VALUES SAVED IN Y
for i=1:length(X)
    Y(1,i+1)=X(i);            
end
%disp(del_o);
%implementing zig-zag
global del_st;
del_st=-10*pi/180.0;


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
    %bot will turn opposite to rudder deflection
    if (X(11)>10*pi/180.0) 
       del_st=10*pi/180.0;
    else if (X(11)<-10*pi/180.0)
       del_st=-10*pi/180.0;
        end
    end
   end 
   r2d = 180/pi;
   %disp(Y(:,13));
   

   figure(1);
   plot(t, Y(:,14));xlabel('time');ylabel('\delta_{s_{actual}}');
   saveas(figure(1),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_stern\const_const\delstern.eps');
   
   figure(2);
   subplot(3,1,1);plot(t,Y(:,12)*r2d);xlabel('time');ylabel('\theta');
   subplot(3,1,2);plot(t,Y(:,11)*r2d);xlabel('time');ylabel('\phi');
   subplot(3,1,3);plot(t,Y(:,13)*r2d);xlabel('time');ylabel('\psi');
   saveas(figure(2),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_stern\const_const\euler', 'eps');
   
   figure(3);
   subplot(3,1,1);plot(t,Y(:,8)); xlabel('time');ylabel('X');
   subplot(3,1,2);plot(t,Y(:,9)); xlabel('time');ylabel('Y');
   subplot(3,1,3);plot(t,Y(:,10)); xlabel('time');ylabel('Z');
   saveas(figure(3),'E:\AUV\AUVForwardDynamics\Results\vert_zigzag_stern\const_const\xyz', 'eps');
   
%    hold on
%    plot(t,Y(:,14))
   %plot(t,Y(:,11));
end