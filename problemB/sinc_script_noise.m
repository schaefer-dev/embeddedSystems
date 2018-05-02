rng(13);

t_o = linspace(0,2,100);
f_original = sin(2*pi*t_o + 1/4) + sin(4 * pi * t_o + 1/4);


% MANUAL: ----------
% comment in frequency + t_s blocks to plot the corresponding frequency,
% ugly copy&paste, but works for now

% frequency = 1/8;
% t_s = [0*frequency, 1*frequency, 2*frequency, 3*frequency, 4*frequency, 5*frequency, 6*frequency, 7*frequency, 8*frequency, 9*frequency, 10*frequency, 11*frequency, 12*frequency, 13*frequency, 14*frequency, 15*frequency, 16*frequency];

% frequency = 1/3;
% t_s = [0*frequency, 1*frequency, 2*frequency, 3*frequency, 4*frequency, 5*frequency, 6*frequency];

% frequency = 1/4;
% t_s = [0*frequency, 1*frequency, 2*frequency, 3*frequency, 4*frequency, 5*frequency, 6*frequency, 7*frequency, 8*frequency];

 frequency = 1/5;
 t_s = [0*frequency, 1*frequency, 2*frequency, 3*frequency, 4*frequency, 5*frequency, 6*frequency, 7*frequency, 8*frequency, 9*frequency, 10*frequency];

 
% Parameters used for random
eps_scale = 0.2;
 
 

f_sampled = sin(2*pi*t_s + 1/4) + sin(4 * pi * t_s + 1/4);


v0 = sample_function(0*frequency) + (rand - 0.5) * eps_scale;
v1 = sample_function(1*frequency) + (rand - 0.5) * eps_scale;
v2 = sample_function(2*frequency) + (rand - 0.5) * eps_scale;
v3 = sample_function(3*frequency) + (rand - 0.5) * eps_scale;
v4 = sample_function(4*frequency) + (rand - 0.5) * eps_scale;
v5 = sample_function(5*frequency) + (rand - 0.5) * eps_scale;
v6 = sample_function(6*frequency) + (rand - 0.5) * eps_scale;
v7 = sample_function(7*frequency) + (rand - 0.5) * eps_scale;
v8 = sample_function(8*frequency) + (rand - 0.5) * eps_scale;
v9 = sample_function(9*frequency) + (rand - 0.5) * eps_scale;
v10 = sample_function(10*frequency) + (rand - 0.5) * eps_scale;
v11 = sample_function(11*frequency) + (rand - 0.5) * eps_scale;
v12 = sample_function(12*frequency) + (rand - 0.5) * eps_scale;
v13 = sample_function(13*frequency) + (rand - 0.5) * eps_scale;
v14 = sample_function(14*frequency) + (rand - 0.5) * eps_scale;
v15 = sample_function(15*frequency) + (rand - 0.5) * eps_scale;
v16 = sample_function(16*frequency) + (rand - 0.5) * eps_scale;

% sinc computation looks strange!

syms sinc(i)
sinc(i) = piecewise(abs(i) > 0, sin((1/frequency)*pi*i)/((1/frequency)*pi*i), i==0, 1.0);
g = v0 * sinc(t_o - 0*frequency) + v1 * sinc(t_o - 1*frequency) + v2 * sinc(t_o - 2*frequency) + v3 * sinc(t_o - 3*frequency) + v4 * sinc(t_o - 4*frequency) + v5 * sinc(t_o - 5*frequency) + v6 * sinc(t_o - 6*frequency) + v7 * sinc(t_o - 7*frequency) + v8 * sinc(t_o - 8*frequency)+ v9 * sinc(t_o - 9*frequency)+ v10 * sinc(t_o - 10*frequency)+ v11 * sinc(t_o - 11*frequency)+ v12 * sinc(t_o - 12*frequency)+ v13 * sinc(t_o - 13*frequency)+ v14 * sinc(t_o - 14*frequency)+ v15 * sinc(t_o - 15*frequency)+ v16 * sinc(t_o - 16*frequency);

plot(t_o, g); hold on; plot(t_o, f_original); legend('g with e=0.1,f=1/5','original signal');



function y = sample_function(x)
y = sin(2*pi*x + 1/4) + sin(4 * pi * x + 1/4);
end