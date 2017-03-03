function  a_b_valuesArray = check_eigen(A,B,k4)
% calculates and returns an array containing values of a and b(gains) for
% which eigenvalues of B are all less than zero

%a_b_valuesArray =zeros(100, 2);
ctr =1;



for i= 0 : .1 : 10
    for j = 1 : .1 : 10
        A(3,4) = j/k4;%b
        B(3,4) = i;%a
        e(1,:) = eig(A\B);
    
        flag = 0;
        r=length(e);
        
        for k=1:r
            if(e(k)> 0 || e(k)==0 || isnan(e(k)) ),flag = 1; break; end

        end    
        
        if(flag==0)
           temp(ctr,:) = [i j e(1,:)];
           
           ctr=ctr+1;
        end
    end


end

a_b_valuesArray = sortrows(temp,-3);



end