t_o = linspace(0,2,100);
f_original = sin(2*pi*t_o + 1/4) + sin(4 * pi * t_o + 1/4);


% MANUAL: ----------
% comment in frequency + t_s blocks to plot the corresponding frequency,
% ugly copy&paste, but works for now

% frequency = 1/8;
% t_s = [0*frequency, 1*frequency, 2*frequency, 3*frequency, 4*frequency, 5*frequency, 6*frequency, 7*frequency, 8*frequency, 9*frequency, 10*frequency, 11*frequency, 12*frequency, 13*frequency, 14*frequency, 15*frequency, 16*frequency];

% frequency = 1/4;
% t_s = [0*frequency, 1*frequency, 2*frequency, 3*frequency, 4*frequency, 5*frequency, 6*frequency, 7*frequency, 8*frequency];

frequency = 1/5;
t_s = [0*frequency, 1*frequency, 2*frequency, 3*frequency, 4*frequency, 5*frequency, 6*frequency, 7*frequency, 8*frequency, 9*frequency, 10*frequency];


f_sampled = sin(2*pi*t_s + 1/4) + sin(4 * pi * t_s + 1/4);

v0 = sin(2*pi* 0 * frequency + 1/4) + sin(4 * pi * 0 * frequency + 1/4);
v1 = sin(2*pi* 1 * frequency + 1/4) + sin(4 * pi * 1 * frequency + 1/4);
v2 = sin(2*pi* 2 * frequency + 1/4) + sin(4 * pi * 2 * frequency + 1/4);
v3 = sin(2*pi* 3 * frequency + 1/4) + sin(4 * pi * 3 * frequency + 1/4);
v4 = sin(2*pi* 4 * frequency + 1/4) + sin(4 * pi * 4 * frequency + 1/4);
v5 = sin(2*pi* 5 * frequency + 1/4) + sin(4 * pi * 5 * frequency + 1/4);
v6 = sin(2*pi* 6 * frequency + 1/4) + sin(4 * pi * 6 * frequency + 1/4);
v7 = sin(2*pi* 7 * frequency + 1/4) + sin(4 * pi * 7 * frequency + 1/4);
v8 = sin(2*pi* 8 * frequency + 1/4) + sin(4 * pi * 8 * frequency + 1/4);
v9 = sin(2*pi* 9 * frequency + 1/4) + sin(4 * pi * 9 * frequency + 1/4);
v10 = sin(2*pi* 10 * frequency + 1/4) + sin(4 * pi * 10 * frequency + 1/4);
v11 = sin(2*pi* 11 * frequency + 1/4) + sin(4 * pi * 11 * frequency + 1/4);
v12 = sin(2*pi* 12 * frequency + 1/4) + sin(4 * pi * 12 * frequency + 1/4);
v13 = sin(2*pi* 13 * frequency + 1/4) + sin(4 * pi * 13 * frequency + 1/4);
v14 = sin(2*pi* 14 * frequency + 1/4) + sin(4 * pi * 14 * frequency + 1/4);
v15 = sin(2*pi* 15 * frequency + 1/4) + sin(4 * pi * 15 * frequency + 1/4);
v16 = sin(2*pi* 16 * frequency + 1/4) + sin(4 * pi * 16 * frequency + 1/4);


syms sinc(i)
sinc(i) = piecewise(abs(i) > 0, sin((1/frequency)*pi*i)/((1/frequency)*pi*i), i==0, 1.0);
g = v0 * sinc(t_o - 0*frequency) + v1 * sinc(t_o - 1*frequency) + v2 * sinc(t_o - 2*frequency) + v3 * sinc(t_o - 3*frequency) + v4 * sinc(t_o - 4*frequency) + v5 * sinc(t_o - 5*frequency) + v6 * sinc(t_o - 6*frequency) + v7 * sinc(t_o - 7*frequency) + v8 * sinc(t_o - 8*frequency)+ v9 * sinc(t_o - 9*frequency)+ v10 * sinc(t_o - 10*frequency)+ v11 * sinc(t_o - 11*frequency)+ v12 * sinc(t_o - 12*frequency)+ v13 * sinc(t_o - 13*frequency)+ v14 * sinc(t_o - 14*frequency)+ v15 * sinc(t_o - 15*frequency)+ v16 * sinc(t_o - 16*frequency);

plot(t_o, g); hold on; plot(t_o, f_original); stairs(t_s, f_sampled) ;legend('g','signal', 'output');