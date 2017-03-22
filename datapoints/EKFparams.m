global sigma_a sigma_g sigma_apg P Q R tinc DVL PSENSOR

sigma_a = accelerometer_noise_density*(1/sqrt(tinc));
sigma_g = gyroscope_noise_density*(1/sqrt(tinc));
sigma_apg = sigma_a + sigma_g; %sqrt(sigma_a*sigma_a + sigma_g*sigma_g); % TODO : Not confirm
P = diag([0.05,0.05,0.05, 0.05,0.05,0.05, 0.05,0.05,0.05]);
Q = diag([0,0,0, sigma_g, sigma_g, sigma_g, sigma_apg, sigma_apg, sigma_apg]);
R = diag([0.05, 0.00087, 0.05]);

DVL = 1;
PSENSOR = 0;