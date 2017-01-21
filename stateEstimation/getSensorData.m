function [ Y ] = getSensorData(X)
%GETSENSORDATA outputs DVL and pressure sensor data.

H = zeros(9,3);
H(7:9,:) = eye(3,3);

Y = H'*X;

end

