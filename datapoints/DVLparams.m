global dvl_location DVL_to_body dvl_noise_density;

dvl_location = [-100; 20; 300]*1e-3; %[m]

DVL_to_body = [ 1  0  0
                0  1  0
                0  0  1];
            
dvl_noise_density =  0.015; % 0.015 (m/s)/rt(Hz) 1 sigma (continuous)
