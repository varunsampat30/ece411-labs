clear all
clc
%% Set up
k = 1;
g = 9.8;

%% Linearize model

x_bar = [0.5; 0.0];
u_bar = sqrt(g*x_bar(1)^2);

lin_sys     = linmod('non_linear_model', x_bar, u_bar);
[num, den]  = ss2tf(lin_sys.a, lin_sys.b, lin_sys.c, lin_sys.d);

A = lin_sys.a;
B = lin_sys.b;
%% Pole placement
p_ctrl = [-1 -1.5];
F      = -1*place(A, B, p_ctrl) % we use u = Kx instead of u = -Kx
f1 = F(1);
f2 = F(2);

alpha = 10;
c = f1 + f2*alpha;
d = f1 * alpha

% (cs + d)/(s + alpha)
sys = tf([c, d],[1 alpha])

%% dt sampling period
T  = 0.11;

%% 4.2.3 Discretize first order output controller
discrete_controller = c2d(sys, T, 'tustin')
[d_num,d_den,ts] = tfdata(discrete_controller);
d_num = cell2mat(d_num);
d_den = cell2mat(d_den);
