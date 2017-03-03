
function Y=eulerFirstOrder(forwarddynamics2,timespan,X, ord_defl,caseNo)
% Function for solving 6DOF equations using 1st order euler method


h = diff(timespan);% for obtaining time step
dt = h(1);

Y = zeros(length(timespan),length(X)+7);
Y(:,1) = timespan(:);
Y(1,2:end -6) = X(:);


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
  
  
      if strcmp(caseNo,'7') || strcmp(caseNo,'8')|| strcmp(caseNo,'13') || strcmp(caseNo,'14')
           if (yi(11)>abs(ord_defl(2))) 
            ord_defl(2)=abs(ord_defl(2));
           elseif (yi(11)<-abs(ord_defl(2)))
            ord_defl(2)=-abs(ord_defl(2));    
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
    

      
      
 DX=forwarddynamics2(timespan(i),X,ord_defl,caseNo,D);
    temp=X+DX*dt;
    X=temp;
        
  for j=1:length(X)
      Y(i,j+1)=X(j);
  end
  
  for k = 1:6
      Y(i,length(X)+1+k) = DX(k);
  end
  
end
 


    
 end
 
 
 
 