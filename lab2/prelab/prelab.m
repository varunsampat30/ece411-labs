%% Set up
k = 1;
g = 9.8;

syms x1 x2 u
%% Linearize model
x_bar = [0.5; 0.0];
u_bar = sqrt(g*x_bar(1)^2);

lin_sys = linmod('non_linear_model', x_bar, u_bar);

[num, den] = ss2tf(sys.a, sys.b, sys.c, sys.d)
