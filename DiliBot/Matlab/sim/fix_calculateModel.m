%% @file re_calculateModel.m
%% @brief This file re-calculating the simulation model of ballbot

clear all, close all, clc

%% Ballbot parameter
rk = 0.1225; % [m] ban kinh bong
rw = 0.029;  % [m] ban kinh ominiwheel
rA = 0.1;    % [m] ban kinh than robot

%l = 0.2;     % [m] khoang cach tu tam robot den tam bong
l = 0.2;

mk = 5;    % [kg] khoi luong bong
mw = 0.06;   % [kg] khoi luong banh xe da huong
mA = 2.5;    % [kg] khoi luong than robot

g = 9.81;    % [m/s^2] gia toc trong truong

alpha = pi/4;% [rad] goc giua banh xe va bong
beta  = 0;   % [rad] goc giua motor 1 va Ox

i = 33;             % [] ti so truyen cua hop so
Im = 3.33*10^(-6);  % [kg.m^2] momen quan tinh cua dong co

disp('(*)Calculating model parameter for simulation...')

%% Tinh momen quan tinh
Ik  = (2/3)*mk*rk^(2); % cau rong dong chat   - quan tinh bong
Iw  = (1/2)*mw*rw^(2); % dia tron dong chat   - quan tinh banh xe
Iwx = (3/2)*((cos(alpha))^2*(Iw + (i^2)*Im)); % quan tinh banh xe ao
IA  = (1/2)*mA*rA^2;   % tru rong - FIXME cai nay sai so lon!

%% Tinh toan mo hinh
syms thetax thetax_dot phix phix_dot Tx; % Tx = u <- control

% M11 = const - (M11)
M11 = (mk+mA+mw)*(rk^2) + Ik + ((rk/rw)^2)*Iwx;

% M12 = M12a * cos(theta) + M12b
M12 = -(rk/(rw^2))*(rk+rw)*Iwx + (l*mA + (rk+rw)*mw)*rk*cos(thetax);
M12a = (l*mA + (rk+rw)*mw)*rk;
M12b = -(rk/(rw^2))*(rk+rw)*Iwx;

% M21 = M21a * cos(theta) + M21b
M21 = -(rk/(rw^2))*(rk+rw)*Iwx + (l*mA + (rk+rw)*mw)*rk*cos(thetax);
M21a = (l*mA + (rk+rw)*mw)*rk;
M21b = -(rk/(rw^2))*(rk+rw)*Iwx;

% M22 = const - (M22)
M22 = (((rk+rw)/rw)^2)*Iwx + IA + mA*l^2 + mw*(rk+rw)^2;

M11 = vpa(M11);
M12 = vpa(M12);
M21 = vpa(M21);
M22 = vpa(M22);

M11 = double(M11);
M22 = double(M22);

Mx = [M11 M12; M21 M22];

% C11 = C11a*sin(thetax)*(thetax_dot)^2
C11  = -rk*(l*mA + (rk+rw)*mw)*sin(thetax)*(thetax_dot)^2;
C11a = -rk*(l*mA + (rk+rw)*mw);

C11 = vpa(C11);
C21 = 0;
Cx = [C11; C21];

G11 = 0;
G21  = -g*(l*mA + (rk+rw)*mw)*sin(thetax);
G21a = -g*(l*mA + (rk+rw)*mw);
G21 = vpa(G21);
Gx = [G11; G21];

F11  = -(rk/rw)*Tx;
F11a = (-rk/rw);
F11  = vpa(F11);

F21  = -(rk/rw)*Tx;
F21a = (-rk/rw);
F21  = vpa(F21);
Fx   = [F11; F21];


% Manual calculate inv(Mx)
det_Mx = M11*M22 - (M21a*cos(thetax) + M21b)^2;

inv_Mx_11 = (1/det_Mx) * M22;
inv_Mx_12 = (1/det_Mx) * (-(M21a*cos(thetax) + M21b));
inv_Mx_21 = (1/det_Mx) * (-(M12a*cos(thetax) + M12b));
inv_Mx_22 = (1/det_Mx) * M11;

calc_inv_Mx = [inv_Mx_11 inv_Mx_12; inv_Mx_21 inv_Mx_22];

% Checking by subs:
for i = 0:0.5:10
    check_theta = subs(inv(Mx), thetax, i) - subs(calc_inv_Mx, thetax, i);
    check_theta = round(check_theta);
end

FCG_11 =  -(rk/rw)*Tx - C11a*sin(thetax)*(thetax_dot)^2;
FCG_21 =  -(rk/rw)*Tx - G21a*sin(thetax);

FCG = [FCG_11; FCG_21];

% *Note: this equation is manual calculate for simulink
% F11_sys meaning: phix_dot_dot equation
F11_sys = (M22*(-(rk/rw)*Tx - C11a*sin(thetax)*(thetax_dot)^2) ...
           - (M21a*cos(thetax) + M21b)*(-(rk/rw)*Tx - G21a*sin(thetax))) / ...
          (M11*M22 - (M21a*cos(thetax) + M21b)^2);
F21_sys = (-(M21a*cos(thetax) + M21b)*(-(rk/rw)*Tx - C11a*sin(thetax)*(thetax_dot)^2) ...
           + M11*(-(rk/rw)*Tx - G21a*sin(thetax))) / ...
          (M11*M22 - (M21a*cos(thetax) + M21b)^2);

F_sys_manual = [F11_sys; F21_sys];
F_sys_manual = vpa(F_sys_manual);

F_sys_simple = inv(Mx)*(Fx - Cx -Gx);
F_sys_simple = vpa(F_sys_simple);

% Checking F_sys
for i = 0:0.5:10
    check_theta = subs(subs(subs(F_sys_manual, thetax, i), thetax_dot, i+1),Tx, i*5) - ...
                  subs(subs(subs(F_sys_simple, thetax, i), thetax_dot, i+1),Tx, i*5);
    check_theta = round(check_theta);
    if isequal(check_theta, [0;0])
        disp('Check F_sys ok');
    else
        disp('Check F_sys Error!!!');
    end
end

disp('It is ok for calling controller calculation and simulation (^^)');
