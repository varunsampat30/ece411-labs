clear all

%% vars

A = [0          1.0000;39.2000         0];
B = [0; -12.5220];
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

T  = 0.05;
