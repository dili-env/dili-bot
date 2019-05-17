%% @file lqr_control.m
%% @brief Linearing system and calculate LQR controller
%  *Note: This file have to be called after model calculation completed

clc

Alpha = pi/4;
Beta = 0;

%% Linearization system model
f1 = phix_dot;
f2 = thetax_dot;
f3 = F_sys_simple(1,1);
f4 = F_sys_simple(2,1);

%% Jacobi expression
A11 = diff(f1,phix);
A12 = diff(f1,thetax);
A13 = diff(f1,phix_dot);
A14 = diff(f1,thetax_dot);

A21 = diff(f2,phix);
A22 = diff(f2,thetax);
A23 = diff(f2,phix_dot);
A24 = diff(f2,thetax_dot);

A31 = diff(f3,phix);
A32 = diff(f3,thetax);
A33 = diff(f3,phix_dot);
A34 = diff(f3,thetax_dot);

A41 = diff(f4,phix);
A42 = diff(f4,thetax);
A43 = diff(f4,phix_dot);
A44 = diff(f4,thetax_dot);

B11 = diff(f1, Tx); 
B21 = diff(f2, Tx); 
B31 = diff(f3, Tx); 
B41 = diff(f4, Tx); 

disp('==================================');
disp('Half system state:');
disp('[phix; thetax; phix_dot; thetax_dot]')
A = [A11 A12 A13 A14; 
     A21 A22 A21 A24;
     A31 A32 A33 A34;
     A41 A41 A43 A44];

B = [B11; B21; B31; B41];

disp('==================================');
disp('Manual calculation Jacobi result:');
A32_tmp = subs(A32, thetax, 0);
A_32_cal = vpa(subs(A32_tmp,thetax_dot,0));
A34_tmp = subs(A34, thetax, 0);
A_34_cal = vpa(subs(A34_tmp,thetax_dot,0));
A42_tmp = subs(A42, thetax, 0);
A_42_cal = vpa(subs(A42_tmp,thetax_dot,0));

B31_tmp = subs(B31, thetax, 0);
B_31_cal = vpa(subs(B31_tmp,thetax_dot,0));
B41_tmp = subs(B41, thetax, 0);
B_41_cal = vpa(subs(B41_tmp,thetax_dot,0));

%% For real system
A_32 = round(double(A_32_cal),4);
A_34 = 0;
A_42 = round(double(A_42_cal),4);

B_31 = round(double(B_31_cal),4);
B_41 = round(double(B_41_cal),4);

%% LQR calculation for discreate time system:
% x = [phix thetax phix_dot thetax_dot]
A_tot = [0 0    1 0;
         0 0    0 1;
         0 A_32 0 A_34;
         0 A_42 0 0]
B_tot = [0; 0; B_31; B_41]
C_tot = [1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1]
D_tot = [0; 0; 0; 0]

Q = diag([20 5000 10 5]);
 
R = 1;

 %% Discreate time LQR calculation
sys = ss(A_tot, B_tot, C_tot, D_tot)
dsys=c2d(sys, 0.005, 'zoh')
K_tot = dlqr(dsys.a, dsys.b, Q, R)
