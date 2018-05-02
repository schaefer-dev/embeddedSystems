t_o = linspace(0,2,100);
f_original = sin(2*pi*t_o + 1/4) + sin(4 * pi * t_o + 1/4);

t_s = [1/4, 2/4, 3/4, 4/4, 5/4, 6/4, 7/4, 8/4];
f_sampled = sin(2*pi*t_s + 1/4) + sin(4 * pi * t_s + 1/4);

frequency = 1/4;

v0 = sin(2*pi* 0 * frequency + 1/4) + sin(4 * pi * 0 * frequency + 1/4);
v1 = sin(2*pi* 1 * frequency + 1/4) + sin(4 * pi * 1 * frequency + 1/4);
v2 = sin(2*pi* 2 * frequency + 1/4) + sin(4 * pi * 2 * frequency + 1/4);
v3 = sin(2*pi* 3 * frequency + 1/4) + sin(4 * pi * 3 * frequency + 1/4);
v4 = sin(2*pi* 4 * frequency + 1/4) + sin(4 * pi * 4 * frequency + 1/4);
v5 = sin(2*pi* 5 * frequency + 1/4) + sin(4 * pi * 5 * frequency + 1/4);
v6 = sin(2*pi* 6 * frequency + 1/4) + sin(4 * pi * 6 * frequency + 1/4);
v7 = sin(2*pi* 7 * frequency + 1/4) + sin(4 * pi * 7 * frequency + 1/4);
v8 = sin(2*pi* 8 * frequency + 1/4) + sin(4 * pi * 8 * frequency + 1/4);


syms sinc(i)
sinc(i) = piecewise(abs(i) > 0, sin(4*pi*i)/(4*pi*i), i==0, 1.0);
g = v0 * sinc(t_o - 0*frequency) + v1 * sinc(t_o - 1*frequency) + v3 * sinc(t_o - 3*frequency) + v4 * sinc(t_o - 4*frequency) + v5 * sinc(t_o - 5*frequency) + v7 * sinc(t_o - 7*frequency) + v8 * sinc(t_o - 8*frequency);

plot(t_o, g); hold on; plot(t_o, f_original); stairs(t_s, f_sampled);