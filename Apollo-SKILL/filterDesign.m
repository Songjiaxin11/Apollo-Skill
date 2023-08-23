clc;
clear;

% All frequency values are in Hz.
Fs = 100;  % Sampling Frequency

N  = 2;    % Order 2阶
Fc = 10;  % Cutoff Frequency 截止频率 高频响应性 低频稳定性

% Construct an FDESIGN object and call its BUTTER method.
h  = fdesign.lowpass('N,F3dB', N, Fc, Fs);
Hd = design(h, 'butter');

data = readmatrix("data.xlsx");
L = data(1:500, 1:6);
R = data(1:500, 7:12);
t = 0.01:0.01:5;

fs = 10;
LHighClock = L(:, 2);%被滤波的数据
N = length(LHighClock);
f_0 = fs/N;
F_LHighClock = fft(LHighClock);
F_LHighClock = F_LHighClock / N;
F_LHighClock = 2 * abs(F_LHighClock);
F_LHighClock = F_LHighClock(1:N/2+1, :);
f = f_0 * (0 : N/2);
% plot(f, F_LHighClock);

plot(t, LHighClock);
hold on;
% filt_L = filter(Hd, LHighClock);
% plot(t, filt_L);
% hold on;

[bb, aa] = sos2tf(Hd.sosMatrix, Hd.ScaleValues)

filt_L = filter(bb, aa, LHighClock);
plot(t, filt_L);

