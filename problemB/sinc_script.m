t_o = linspace(0,2,100);
f_original = sin(2*pi*t_o + 1/4) + sin(4 * pi * t_o + 1/4);

t_s = [1/4, 2/4, 3/4, 4/4, 5/4, 6/4, 7/4, 8/4];
f_sampled = sin(2*pi*t_s + 1/4) + sin(4 * pi * t_s + 1/4);

syms sinc(i)
sinc(i) = piecewise(abs(i) > 0, sin(4*pi*i)/(4*pi*i), i==0, 1.0);
g = 0.5 * sinc(t_o-0) + 0.72 * sinc(t_o-1/4) - 1.22 * sinc(t_o-3/4) + 0.5 * sinc(t_o-4/4) + 0.72 * sinc(t_o-5/4) - 1.22 * sinc(t_o-7/4) + 0.5 * sinc(t_o-8/4);

plot(t_o, g); hold on; plot(t_o, f_original); stairs(t_s, f_sampled);