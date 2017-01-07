function Y = rk4t(F,tspan,y0,ord_defl,caseNo)

h = diff(tspan);
% y0 = y0(:);   % Make a column vector.
nrows = length(y0);
N = length(tspan);
Y = zeros(nrows,N);
K = zeros(nrows,4);

        
       
Y(:,1) = y0;
for i = 2:N
  ti = tspan(i-1);
  hi = h(i-1);
  yi = Y(:,i-1);
  
  if caseNo == '3' || caseNo == '4' || caseNo == '5' || caseNo == '6' 
      if (yi(12)>abs(ord_defl(1))) 
            ord_defl(1)=abs(ord_defl(1));
        else if (yi(12)<-abs(ord_defl(1)))
            ord_defl(1)=-abs(ord_defl(1));    
            end
       end
  end
  
  if caseNo == '7' || caseNo == '8'
      if (yi(11)>abs(ord_defl(2))) 
            ord_defl(2)=abs(ord_defl(2));
        else if (yi(11)<-abs(ord_defl(2)))
            ord_defl(2)=-abs(ord_defl(2));    
            end
      end
  end
  
  %Y(i,1) = tspan(i-1);
  K(:,1) = feval(F,ti,yi,ord_defl,caseNo);
  K(:,2) = feval(F,ti+0.5*hi,yi+0.5*hi*K(:,1),ord_defl,caseNo);
  K(:,3) = feval(F,ti+0.5*hi,yi+0.5*hi*K(:,2),ord_defl,caseNo);  
  K(:,4) = feval(F,tspan(i),yi+hi*K(:,3),ord_defl,caseNo);
  Y(:,i) = yi + (hi/6)*(K(:,1) + 2*K(:,2) + 2*K(:,3) + K(:,4));
end
Y = Y.';
buffer(:,1) = tspan;
for iter=1:nrows%Column
    for j=1:N%Rows
        buffer(j,iter+1) = Y(j,iter);
    end
end
Y = buffer;
end