%% @file test_filter
%% @brief simple lowpass filter check
%Note: Please run signal_process before calling this file

close all; clc;
%% Get value
figure('name', 'Manual alpha filter');
s(1) = subplot(2,2,1);
plot(s(1), t, ax);
title('Raw ax data');
s(2) = subplot(2,2,3);
plot(s(2), t, gx);
title('Raw gx data');


%% Filter check
CUTOFF = 5;
RC = 1/(CUTOFF * 2*pi);
fs = 200;
dt = 1/fs;
n = length(ax);
alpha = dt/(RC+dt);

ax_out = zeros(size(ax));
gx_out = zeros(size(gx));

ax_out(1) = ax(1);
gx_out(1) = gx(1);

for i=2:n
    ax_out(i) = ax_out(i-1) + alpha*(ax(i) - ax_out(i-1));
    gx_out(i) = gx_out(i-1) + alpha*(gx(i) - gx_out(i-1));
end

%% Checking angle rate by hand
pre_ax = zeros(size(ax));
pre_ax(1) = ax(1);
for i=2:n
    pre_ax(i) = (ax(i)-ax(i-1))/dt;
end

%% Filter for manual ax
gx_manual = zeros(size(ax));
gx_manaul(1) = pre_ax(1);
for i=2:n
    gx_manual(i) = gx_manual(i-1) + alpha*(pre_ax(i) - gx_manual(i-1));
end

%% Plot checking
s(3) = subplot(2,2,2);
plot(s(3), t, ax_out);
title('Filtered ax data');
s(4) = subplot(2,2,4);
plot(s(4), t, gx_out);
title('Filtered gx data and manual d(ax)');
plot(t, -gx_out, 'b', t, gx_manual, 'r');
grid on