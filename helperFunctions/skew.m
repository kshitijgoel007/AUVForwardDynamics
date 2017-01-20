function [ skew_mat ] = skew( vector )
%SKEW returns skew symmetric form of a vector.

skew_mat = [ 0         -vector(3)  vector(2);
             vector(3) 0          -vector(1);
            -vector(2) vector(1)   0 ];
end
