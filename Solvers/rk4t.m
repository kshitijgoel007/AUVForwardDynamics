function [Y, X_estimate, P_estimate] = rk4t(F,tspan,y0,ord_defl,caseNo)
h = diff(tspan);
global tinc;

% y0 = y0(:);   % Make a column vector.
nrows = length(y0);
N = length(tspan);
Y = zeros(nrows+6,N);
K = zeros(nrows,4);
       
Y(1:18,1) = y0;

% initialising disturbance function
D = zeros(length(y0),1);
X_estimate(1,:) = zeros(9,1);
P_estimate(1,:,:) = zeros(9,9);
%*********************SOLVING******************************************

for i = 2:N
  
  ti = tspan(i-1);
  hi = h(i-1);
  yi = Y(1:18,i-1);
  
  
  
  
  if strcmp(caseNo,'3') || strcmp(caseNo,'4') ||strcmp(caseNo,'5')||strcmp(caseNo,'6') 
      if (yi(12)>abs(ord_defl(1))) 
            ord_defl(1)=abs(ord_defl(1));
      elseif (yi(12)<-abs(ord_defl(1)))
            ord_defl(1)=-abs(ord_defl(1));    
      end
  end
  
  
  if strcmp(caseNo,'7') || strcmp(caseNo,'8')|| strcmp(caseNo,'13') || strcmp(caseNo,'14')
      if (yi(11)>abs(ord_defl(2))) 
            ord_defl(2)=abs(ord_defl(2));
        else if (yi(11)<-abs(ord_defl(2)))
            ord_defl(2)=-abs(ord_defl(2));    
            end
      end
  end
  
   
  if strcmp(caseNo,'9')
      
      % giving external disturbance force 
      if timespan(i) >=5 && timespan(i)<10 
          D(:) = zeros();
          D(6,1) = .5; % r disturbance of .5
     
      elseif  timespan(i)> 100 && timespan(i)<110
          D(:) = zeros();
          D(2,1) = .25; % v disturbance of .25;
          
      elseif timespan(i)> 200 && timespan(i)<210
          D(:) = zeros();
          D(2,1) = .2; 
          D(6,1) = .4;
          
      else
          D(:) = zeros();
      
      end
 end
  
  
  
  if strcmp(caseNo,'10')
  
  
      % giving external disturbance force 
      if timespan(i) >=5 && timespan(i)<10 
          D(:) = zeros();
          D(3,1) = .5; % w disturbance of .5
     
      elseif  timespan(i)> 100 && timespan(i)<110
          D(:) = zeros();
          D(5,1) = .25; % q disturbance of .25;
          
      elseif timespan(i)> 200 && timespan(i)<210
          D(:) = zeros();
          D(3,1) = .2; 
          D(5,1) = .4;
          
      else
          D(:) = zeros();
      
      end
  end

      
  %Y(i,1) = tspan(i-1);
  K(:,1) = feval(F,ti,yi,ord_defl,caseNo,D);
  K(:,2) = feval(F,ti+0.5*hi,yi+0.5*hi*K(:,1),ord_defl,caseNo,D);
  K(:,3) = feval(F,ti+0.5*hi,yi+0.5*hi*K(:,2),ord_defl,caseNo,D);  
  K(:,4) = feval(F,tspan(i),yi+hi*K(:,3),ord_defl,caseNo,D);
  Y(1:18,i) = yi + (hi/6)*(K(:,1) + 2*K(:,2) + 2*K(:,3) + K(:,4));
  
  %includ acc terms
  c = length(Y(1:18,i));
  Y(c+1:c+6)= K(1:6,1);
   
  [X_estimate(i,:), P_estimate(i,:,:)] = stateEstimation(Y', tinc);  
%   X_estimate(i,:)
end
Y = Y.';
 
buffer(:,1) = tspan;
for iter=1:nrows+6%Column
    for j=1:N%Rows
        buffer(j,iter+1) = Y(j,iter);
    end
end
Y = buffer;
end