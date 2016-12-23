function [WMeas,Bias] = gyr_model(WInput,BiasPrev,tinc)

% Gyroscope bias signal %
% gyroscope_sig_beta = gyroscope_random_walk*sqrt(tinc);
% gyroscope_bias += gyroscope_sig_beta*randn(3,1);
gyroscope_bias = gyroscope_bias_instability*randn(3,1);

% Gyroscope white noise signal %
gyroscope_noise_density_d = gyroscope_noise_density*(1/sqrt(tinc));
gyroscope_white_noise = gyroscope_noise_density_d*randn(3,1);   %  White noise (deg/s)

WMeas = WInput + gyroscope_bias + gyroscope_white_noise;

end
