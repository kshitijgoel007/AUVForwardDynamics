function  a_b_valuesArray = check_eigen_B(A,B)
% calculates and returns an array containing values of a and b(gains) for
% which eigenvalues of B are all less than zero

%a_b_valuesArray =zeros(100, 2);
ctr =1;



for i= 0 : .5 : 10
    for j = 1 : .5 : 10
        B(3,2) = j;%b
        B(3,4) = i;%a
        e = eig(A\B);
    
        flag = 0;
        [r, c]=size(e);
        
        for k=1:r
            if(e(k)> 0 || e(k)==0 ),flag = 1; break; end

        end    
        
        if(flag==0)
           a_b_valuesArray(ctr,:) = [i j];
           %disp(e);
           ctr=ctr+1;
        end
    end

%disp(a_b_valuesArray);
end