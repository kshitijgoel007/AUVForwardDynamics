
  function Y=euler(caseNo,botCoordinatesInBodyFrame,timespan)

% Function for solving 6DOF equations using 1st order euler method
% INPUT : caseno : index no of the type of maneuver to be performed
%         botCoordinatesInbodyFrame : 3185X3 matrix containing (x,y,z)
%                                     coordinates of the bot in body frame
%         timespan : time for which simulation is run
% OUTPUT : plots of Trajectory of bot
%          CSV file containing (X,Y,Z) of bot at different time instants 
%          CSV file containing values of state vector X at different time
%          instants
%          Output vector Y
% function Y=euler(caseNo,botCoordinatesInBodyFrame,timespan)


% timstep
  dt=.1;

% Mat file containing properties of bot
  geoprop;                       

% Initialising state vector
  X=zeros(18,1);

% u=1m/s  
  X(1,1)=1;
    
% Initialising output vector for storing values of state vector against time
% for different time instants
  Y=zeros(length(timespan),length(X)+1);

% First column stores time
  Y(1,1)=0;                      

% INITIAL VALUE SAVED IN Y
  for i=1:length(X)
    Y(1,i+1)=X(i);            
  end

% Initialising ordered control surface deflections array
  orderedControlSurfaceDeflectionArray=zeros(4,1);
% orderedDeflectionsArray(1)=del_ordered_rudder
% orderedDeflectionsArray(2)=del_ordered_stern
% orderedDeflectionsArray(3)=del_ordered_bp
% orderedDeflectionsArray(1)=del_ordered_bs


%**************************************************************************  
  
 

%CASES/////////////////////////////////////////////////

switch caseNo
       
      case {'1'}
      % when del_o is positive, bot turns to port(-Y initially points to port
      % also si is -ve.
      orderedControlSurfaceDeflectionArray(1)=10*pi/180.0;
      Y=constantTurn(caseNo,X,Y,timespan,dt,orderedControlSurfaceDeflectionArray,botCoordinatesInBodyFrame);
   
      case {'2'}
      orderedControlSurfaceDeflectionArray(1)=10*pi/180.0;
      Y=constantTurn(caseNo,X,Y,timespan,dt,orderedControlSurfaceDeflectionArray,botCoordinatesInBodyFrame);
   
      case {'3'}
      orderedControlSurfaceDeflectionArray(1)=-10*pi/180.0;
      Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,orderedControlSurfaceDeflectionArray,botCoordinatesInBodyFrame);
      
      case {'4'}
      orderedControlSurfaceDeflectionArray(1)=10*pi/180.0;
      Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,orderedControlSurfaceDeflectionArray,botCoordinatesInBodyFrame);
    
      case {'5'}
      orderedControlSurfaceDeflectionArray(1)=-20*pi/180.0;
      Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,orderedControlSurfaceDeflectionArray,botCoordinatesInBodyFrame);

      case {'6'}
      orderedControlSurfaceDeflectionArray(1)=20*pi/180.0;
      Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,orderedControlSurfaceDeflectionArray,botCoordinatesInBodyFrame);
    
      case {'7'}
      %vertical zig-zag    
      
      case {'8'}
      %vertical zig-zag    
end


% SAVING Y VECTOR
  fname1=['case_',caseNo,'.csv'] ;
  fid=fopen(fname1,'w') ;
  dlmwrite(fname1,Y);
  fclose(fid);
end
  
%*************************************************************************
%*************************************************************************

  function Y=constantTurn(caseNo,X,Y,timespan,dt,orderedControlSurfaceDeflectionArray,botCoordinatesInBodyFrame)
% Function implementing turn in horizontal plane with constant rudder
% deflection

  for i=2:length(timespan)
      % SOLVING USING EULER METHOD
        DX=forwarddynamics2(timespan(i),X,orderedControlSurfaceDeflectionArray);
        temp=X+DX*dt;
        X=temp;
        Y(i,1)=timespan(i);
        for j=1:length(X)
        Y(i,j+1)=X(j);
        end
      % EULER METHOD IMPLEMENTED 
    
      % UPDATING BOT COORDINATES AND SAVING IN FILE
      % UPDATING COORDINATES EVERY 10 SECONDS
        if rem(timespan(i),5)==0|| i==2   
           plotBotInGlobalFrame(caseNo,X(10),X(11),X(12),X(7),X(8),X(9),botCoordinatesInBodyFrame);
        end
      % BOT COORDINATES UPDATED AND SAVED
  end
  
  % Plotting trajectory
    figure;
    plot(Y(:,8),Y(:,9));
    %legend('constantTurn with del_o=%d',del_o);
    
 end
%*************************************************************************
%*************************************************************************

 function Y=zigzagInHorizontalPlane(caseNo,X,Y,timespan,dt,orderedControlSurfaceDeflectionArray,botCoordinatesInBodyFrame)
 
 for i=2:length(timespan)
      % SOLVING USING EULER METHOD
        DX=forwarddynamics2(timespan(i),X,orderedControlSurfaceDeflectionArray);
        temp=X+DX*dt;
        X=temp;
        Y(i,1)=timespan(i);
        for j=1:length(X)
        Y(i,j+1)=X(j);
        end
      % EULER METHOD IMPLEMENTED
         
        if (X(12)>abs(orderedControlSurfaceDeflectionArray(1))) 
            orderedControlSurfaceDeflectionArray(1)=abs(orderedControlSurfaceDeflectionArray(1));
        else if (X(12)<-abs(orderedControlSurfaceDeflectionArray(1)))
            orderedControlSurfaceDeflectionArray(1)=-abs(orderedControlSurfaceDeflectionArray(1));    
            end
        end
      % NOTE : ruddder deflection to starboard side is -ve
      %        X(12)(yaw) towards starboard side is +ve
               
        %UPDATING BOT COORDINATES AND SAVING IN FILE
        %UPDATE COORDINATES EVERY 5 SECONDS
        if rem(timespan(i),5)==0|| i==2   
        plotBotInGlobalFrame(caseNo,X(10),X(11),X(12),X(7),X(8),X(9),botCoordinatesInBodyFrame);
        end
        %BOT COORDINATES UPDATED AND SAVED
 end
 
  % Plotting trajectory
    figure;
    plot(Y(:,8),Y(:,9));
    %legend('case %d',caseNo);
    
end  