%% @file read_data_plot.m
%% @brief read data from buffer MCU

% *********************************
% *Note: 
% Bug from some where that Gx <->-Gy
% TODO: update in keil program
% *********************************
clc; %close all;

Fs = 200;
T  = 1/Fs;

%% Read data

% Tgyro = readtable('data_2.xlsx');   % Move around x axis
% Tgyro = readtable('data_3.xlsx');   % Positive x axis
% Tgyro = readtable('data_4.xlsx');   % Positive move y axis
% Tgyro = readtable('data_5.xlsx');   % Move around x Positive first
Tgyro = readtable('data_9.xlsx');   % Move around y Positive first
% Ax = Tgyro.Ax;
% Ay = Tgyro.Ay;
% Gx = Tgyro.Gx;
% Gy = Tgyro.Gy;

Ax = Tgyro.Thetax;
Ay = Tgyro.Thetay;
Gx = Tgyro.Theta_dotx;
Gy = Tgyro.Theta_doty;

t =  Tgyro.t;

n = length(t);
%% Manual calculate angle x rate
Ax_rate = zeros(size(Ax));
Ax_rate(1) = Ax(1);
Ay_rate = zeros(size(Ay));
Ay_rate(1) = Ay(1);
for i=2:n
    Ax_rate(i) = (Ax(i) - Ax(i-1))/T;
    Ay_rate(i) = (Ay(i) - Ay(i-1))/T;
end

%% Check Ax, Gx, manual Ax_rate
figure('name', 'Ax - Gx - Ax_rate check');
s(1) = subplot(2, 1, 1);
plot(s(1), t, Ax);
title('Ax imu filtered');
grid on;
s(2) = subplot(2, 1, 2);
plot(s(2), t, Gx, 'b', t, Ax_rate, 'r');
title('Gx gyro filtered - Ax rate');
grid on;

%% Check Ay, Gy, manual Ay_rate
figure('name', 'Ay - Gy - Ay_rate check');
s(1) = subplot(2, 1, 1);
plot(s(1), t, Ay);
title('Ay imu filtered');
grid on;
s(2) = subplot(2, 1, 2);
plot(s(2), t, Gy, 'b', t, Ay_rate, 'r');
title('Gy gyro filtered - Ay rate');
grid on;
