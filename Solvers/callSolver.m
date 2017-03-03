
  function Y=callSolver(caseNo,timespan,X,sCheck)

% Function for solving 6DOF equations using rk 4th order/euler method
%
% INPUT : caseno :                    index no of the type of maneuver to be performed
%         sCheck :                    Solver to use
%                                    
%         timespan :                  simulation run-time
%         X:                          initial state(t = 0)

% OUTPUT : plots of Trajectory of bot
%          CSV file containing (X,Y,Z) of bot at different time instants 
%          CSV file containing values of state vector X at different time
%          instants
%          Output vector Y
%
% function Y=euler(caseNo,botCoordinatesInBodyFrame,timespan)

addpath('actuator dynamics');
addpath('utils');
addpath('PDcontrol');



% Mat file containing properties of bot
  geoprop;                       


% Initialising output vector 
Y=zeros(length(timespan),length(X)+1);

  
  
% Initialising ordered control surface deflections array
  ord_defl=zeros(4,1);
% orderedDeflectionsArray(1)=del_ordered_rudder
% orderedDeflectionsArray(2)=del_ordered_stern
% orderedDeflectionsArray(3)=del_ordered_bp
% orderedDeflectionsArray(1)=del_ordered_bs


%**************************************************************************  
  
 



switch caseNo
       
      case {'1'}
      % when del_o is positive, bot turns to port(-Y initially points to port
      % also si is -ve.
      ord_defl(1)=10*pi/180.0;
      
      
    
      case {'2'}
      ord_defl(1)=15*pi/180.0;
      
   
      case {'3'}
      ord_defl(1)=-10*pi/180.0;
      
      case {'4'}
      ord_defl(1)=10*pi/180.0;
    
    
      case {'5'}
      ord_defl(1)=-20*pi/180.0;
    

      case {'6'}
      ord_defl(1)=20*pi/180.0;

     
      case {'7'}
      ord_defl(2)=-10*pi/180.0;
      
      case {'8'}
      ord_defl(2)=10*pi/180.0;

      
      case { '9'}
      
     % impementation in respective solver functions
      
      case {'10'}
     % impementation in respective solver functions
        
      case {'11'}
      ord_defl(1)=-10*pi/180.0;
      
      case{'12'}
       ord_defl(1)=-15*pi/180.0;   
       
      case{'13'}
        ord_defl(2)=-20*pi/180.0;
        
      case{'14'}
        ord_defl(2)=20*pi/180.0;

end % end of switch



% solving..

if sCheck==1
    Y=rk4t(@forwarddynamics2,timespan,X,ord_defl,caseNo);
elseif sCheck==0
    Y = eulerFirstOrder(@forwarddynamics2,timespan,X, ord_defl,caseNo);
end 




end
 