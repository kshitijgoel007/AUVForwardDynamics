
function Y=euler(forwarddynamics2,timespan,X, ord_defl,caseNo)
% Function for solving 6DOF equations using 1st order euler method


h = diff(timespan);% for obtaining time step
dt = h(1);

Y = zeros(length(timespan),length(X)+1);
Y(:,1) = timespan(:);
Y(1,2:end) = X(:);

% initialising disturbance function
D = zeros(length(X),1);
 

for i=2:length(timespan)
      
       yi = Y(i-1,:);

        
       if strcmp(caseNo,'3') || strcmp(caseNo,'4') ||strcmp(caseNo,'5')||strcmp(caseNo,'6') 
           if (yi(12)>abs(ord_defl(1))) 
            ord_defl(1)=abs(ord_defl(1));
           elseif (yi(12)<-abs(ord_defl(1)))
            ord_defl(1)=-abs(ord_defl(1));    
           end
       end
  
  
      if strcmp(caseNo,'7') || strcmp(caseNo,'8')
           if (yi(11)>abs(ord_defl(2))) 
            ord_defl(2)=abs(ord_defl(2));
           elseif (yi(11)<-abs(ord_defl(2)))
            ord_defl(2)=-abs(ord_defl(2));    
           end
      end
      
    
  if strcmp(caseNo,'9')
      
      % giving external disturbance force between 50 and 75 sec and 
      % 150 and 175 sec
      if timespan(i) >=0 && timespan(i)<1 
          D(:) = zeros();
          D(2,1) = .1; % v disturbance of .5
     
      elseif  timespan(i)> 150 && timespan(i)<175
          D(:) = zeros();
         % D(6,1) = .1; % r disturbance of .5;
     
      else
          
          D(:) = zeros();
      
      end
 end
  
  
  
  if strcmp(caseNo,'10')
  
      if timespan(i) >=0&& timespan(i)<1 
          
         % D(3,1) = .5; % w disturbance of .5
     
      elseif  timespan(i)> 150 && timespan(i)<175
          
          %D(5,1) = .1; % q disturbance of .5;
     
      else
          
          D(:) = zeros();
      
      end
  end
      
 DX=forwarddynamics2(timespan(i),X,ord_defl,caseNo,D);
    temp=X+DX*dt;
    X=temp;
        
  for j=1:length(X)
      Y(i,j+1)=X(j);
         
  end
     
end
 


    
 end
 
 
 
 