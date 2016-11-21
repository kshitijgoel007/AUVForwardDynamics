%script for testing zig-zag
function zigzag()
close all;
t=0:.1:50;
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
global del_o;
del_o=-5*pi/180.0;


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
    if (X(12)>5*pi/180.0) 
       %del_o=10*pi/180.0;
       del_o=0;
    else if (X(12)<-5*pi/180.0)
       del_o=-5*pi/180.0;    
        end
    end
   end 
   r2d = 180/pi;
   disp(Y(:,13));
   figure(1);
   plot(t,Y(:,17));xlabel('time');ylabel('Actual deflection of rudder');   

   figure(2);
   plot(t, Y(:,13)*r2d);xlabel('time');ylabel('Yaw');

   %figure(4);
   %plot(Y(:,8),Y(:,9));
end