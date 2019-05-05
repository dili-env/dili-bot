%% @file signal_process.m
%% @brief Design filtering solution for input signal
% **************************************************
%  Note: Can scale lai gyro !!!
% **************************************************
close all, clear all, clc
%% Import data
Tgyro = readtable('gyro_2.xlsx');
%Tgyro = readtable('gyro_3.xlsx');

%% Plot raw data
t= Tgyro.t;
gx = Tgyro.gyro_x*180/pi;
ax = Tgyro.angle_x*180/pi;
% Raw gyro_x data
figure('name','Raw data');
s(1) = subplot(2,1,1);
plot(s(1), t, gx);
title(s(1), 'raw gyro x');
grid on;
xlabel('time(s)');
ylabel('x angle rate(deg/s)');
% Raw angle x data
s(2) = subplot(2,1,2);
plot(s(2), t, ax);
title(s(2), 'raw angle x');
grid on;
xlabel('time(s)');
ylabel('x angle (deg)');

%% Signal spectrial argument
n = length(gx);
fs = 200;
dt = 1/fs;
f = (0:n-1)*(fs/n);

%% Do spectrial analysis for angle x
y_ax = fft(ax);
power_ax = abs(y_ax).^2/n;
%% Plot anglex spectrial
figure('name', 'x angle spectrial');
plot(f, power_ax);

%% Design an alpha filter for angle x (lowpass filter f_cut = 20Hz)
d_ax = fdesign.lowpass('Fp,Fst,Ap,Ast',15,20,0.5,60,200);
Hd_ax = design(d_ax, 'equiripple');
ax_filter = filter(Hd_ax, ax);

%% Plot filter angle x result
figure('name','Angle lowpass filter 20Hz');
s(1) = subplot(2,1,1);
plot(s(1), t, ax);
title(s(1), 'raw angle x');
grid on;
xlabel('time(s)');
ylabel('x angle (deg)');
% Raw angle x data
s(2) = subplot(2,1,2);
plot(s(2), t, ax_filter);
title(s(2), 'filtered angle x');
grid on;
xlabel('time(s)');
ylabel('x angle (deg)');

%% Do spectrial analysis for gyro x
y_gx = fft(gx);
power_gx = abs(y_gx).^2/n;
%% Plot anglex spectrial
figure('name', 'x gyro spectrial');
plot(f, power_gx);

%% Design an alpha filter for angle x (lowpass filter f_cut = 20Hz)
% Same as angle
d_gx_low = fdesign.lowpass('Fp,Fst,Ap,Ast',8,10,1,60,200);
Hd_gx_low = design(d_gx_low, 'equiripple');
gx_filter = filter(Hd_gx_low, gx);

% d_gx_high = fdesign.highpass('Fst,Fp,Ast,Ap',6.5,7,60,0.5,200);
% Hd_gx_high = design(d_gx_high, 'equiripple');
% gx_filter = filter(Hd_gx_high, gx_filter_low);

%% Plot filter angle x result
figure('name','Angle rate lowpass filter 20Hz');
s(1) = subplot(2,1,1);
plot(s(1), t, gx);
title(s(1), 'raw gyro angle rate x');
grid on;
xlabel('time(s)');
ylabel('x angle rate (deg/s)');
% Raw angle x data
s(2) = subplot(2,1,2);
plot(s(2), t, gx_filter);
title(s(2), 'filtered gyro angle rate x');
grid on;
xlabel('time(s)');
ylabel('x angle rate (deg/s)');


%% Check angle rate my calculation
% My stupid calculation
size_ax = size(ax);
pre_ax = zeros(size_ax);
pre_ax_filter = zeros(size_ax);
for i=2:n
    pre_ax(i) = ax(i) - ax(i-1);
    pre_ax_filter(i) = ax_filter(i) - ax_filter(i-1);
end



figure('name','My stupid calculation');
s(1) = subplot(3,2,1);
plot(s(1), t, ax);
title(s(1), 'raw angle x');
grid on;
xlabel('time(s)');
ylabel('x angle (deg/s)');
% Raw angle x data
s(2) = subplot(3,2,2);
plot(s(2), t, pre_ax);
title(s(2), 'raw angle calculate rate x');
grid on;
xlabel('time(s)');
ylabel('x angle rate (deg/s)');

s(3) = subplot(3,2,3);
plot(s(3), t, ax_filter);
title(s(3), 'filtered angle x');
grid on;
xlabel('time(s)');
ylabel('x angle (deg/s)');
% Raw angle x data
s(4) = subplot(3,2,4);
plot(s(4), t, pre_ax_filter);
title(s(4), 'filtered angle rate calculate x');
grid on;
xlabel('time(s)');
ylabel('x angle rate (deg/s)');

% Raw angle rate x data
s(5) = subplot(3,2,5);
plot(s(5), t, gx);
title(s(5), 'Raw gyro angle rate x');
grid on;
xlabel('time(s)');
ylabel('x angle rate (deg/s)');

% Filtered angle rate x data
s(6) = subplot(3,2,6);
plot(s(6), t, gx_filter);
title(s(6), 'Filtered gyro angle rate x');
grid on;
xlabel('time(s)');
ylabel('x angle rate (deg/s)');


%% Final decision compare
figure('name', 'Final signal expected');
% Raw angle x
s(1) = subplot(2,2,1);
plot(s(1), t, ax);
title(s(1), 'raw angle x');
grid on;
xlabel('time(s)');
ylabel('x angle (deg)');
% Raw angle rate x
s(2) = subplot(2,2,2);
plot(s(2), t, gx);
title(s(2), 'gyro raw angle rate x');
grid on;
xlabel('time(s)');
ylabel('gyro x angle rate (deg/s)');

% Filtered angle x
s(3) = subplot(2,2,3);
plot(s(3), t, ax_filter);
title(s(3), 'filtered angle x');
grid on;
xlabel('time(s)');
ylabel('x angle (deg)');
% Filtered angle rate x
s(4) = subplot(2,2,4);
plot(s(4), t, gx_filter);
title(s(4), 'gyro filtered angle rate x');
grid on;
xlabel('time(s)');
ylabel('x angle rate (deg/s)');


%% Final check sign
figure ('name', 'Sign check');
plot(t, pre_ax_filter, 'r', t, gx_filter);
grid on