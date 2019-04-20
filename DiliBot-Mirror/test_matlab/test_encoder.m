%% @file test_encoder.m
%% @brief This file using simple matrix conversion from Psi123 to Psixyz

syms psi_1 psi_2 psi_3 psi_x psi_y psi_z
alpha = pi/4;
N = [cos(alpha)      0                     sin(alpha); ...
     -0.5*cos(alpha) -sqrt(3)/2*cos(alpha) sin(alpha); ...
     -0.5*cos(alpha) sqrt(3)/2*cos(alpha)  sin(alpha)];

N_ = inv(N);
N_ = round(N_, 4)
Psi_123_dot = [psi_1; psi_2; psi_3];
Psi_xyz_dot = [psi_x, psi_y, psi_z];

Psi_xyz_dot = N_*Psi_123_dot