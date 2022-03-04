%% Set up
k = 1;
g = 9.8;

%% Linearize model

x_bar = [0.5; 0.0];
u_bar = sqrt(g*x_bar(1)^2);

lin_sys = linmod('non_linear_model', x_bar, u_bar);

%{
lin_sys.a = [0          1.0000]
            [39.2000         0]

lin_sys.b = [0       ]
            [-12.5220]
%}

[num, den] = ss2tf(lin_sys.a, lin_sys.b, lin_sys.c, lin_sys.d)

%{
num = [0              0  -12.5220]
den = [1.0000         0  -39.2000]
%}
