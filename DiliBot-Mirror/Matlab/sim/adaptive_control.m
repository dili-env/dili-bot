%% @file adaptive_control.m
%% @brief Simple adaptive control for half system
% *Note: Run and check the re-calculate system before calling this file


%% Intitialize data
m11 = M11;
m12 = M12a*cos(thetax) + M12b;
m21 = m12;
m22 = M22;
detM = m11*m22 - m21*m12;

c11 = C11a*sin(thetax)*thetax_dot;
c12 = 0; c21 = 0; c22 = 0;

g11 = 0;
g21 = G21a*sin(thetax);


%% First half system theta related calculate
x1 = [thetax; thetax_dot];
A1 = [0 1; 0  0];
B1 = [0 0; 1 -1];
Lamda1 = [m22/detM 0; 0 m12/detM];
u1 = (rk/rw)*[Tx; Tx];
THETA1 = [0 (m12*C21 - m22*c11)/(-m12); ...
          0 (m12*c22 - m22*c12)/(-m12); ...
          0 (m12*g21 - m22*g11)/(-m12)];
PHI1 = [thetax_dot; phix_dot; 1];

F1_seperate = A1*x1 + B1*Lamda1*(u1 + THETA1'*PHI1)

%% Check theta related first half
F1_seperate21 = F1_seperate(2,:);
for i = 0:0.5:10
    check_theta = subs(subs(subs(F1_seperate21, thetax, i), thetax_dot, i+1),Tx, i*5) - ...
                  subs(subs(subs(F_sys_simple(1,:) , thetax, i), thetax_dot, i+1),Tx, i*5);
    check_theta = round(check_theta);
    if isequal(check_theta, 0)
        disp('Check F_sys1 ok');
    else
        disp('Check F_sys Error!!!');
    end
end


%% Second half theta related calculate
x2 = [phix; phix_dot];
A2 = [0 1; 0  0];
B2 = [0 0; -1 1];
Lamda2 = [m21/detM 0; 0 m11/detM];
u2 = (rk/rw)*[Tx; Tx];
THETA2 = [0 (m21*c11 - m11*c21)/(m11); ...
          0 (m21*c12 - m11*c22)/(m11); ...
          0 (m21*g11 - m11*g21)/(m11)];
PHI2 = [thetax_dot; phix_dot; 1];

F2_seperate = A2*x2 + B2*Lamda2*(u2 + THETA2'*PHI1)

%% Check theta related second half
F2_seperate21 = F2_seperate(2,:);
for i = 0:0.5:10
    check_theta = subs(subs(subs(F2_seperate21, thetax, i), thetax_dot, i+1),Tx, i*5) - ...
                  subs(subs(subs(F_sys_simple(2,:) , thetax, i), thetax_dot, i+1),Tx, i*5);
    check_theta = round(check_theta);
    if isequal(check_theta, 0)
        disp('Check F_sys1 ok');
    else
        disp('Check F_sys Error!!!');
    end
end


%% Calculate reference model
omega = 1;
xi = 0.7;

A_ref = [0 1; -omega^2 -2*xi*omega]
B_ref = [0; omega^2]

Q = [100 0; 0 300];
P = lyap(A_ref', Q)
