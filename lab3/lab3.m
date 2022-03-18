%% ece411: real-time computer control
%  lab3: implementation issues in digital control
%  authors: Pranshu Malik and Varun Sampat
%  date: 18 March 2022


clc;
close all;
clearvars;

%% prelab

% ct plant ss matrices
A = [0 1 0  0   0;
    -1 0 0 -1   0;
     0 0 0  0   0;
     0 0 0  0   1;
     0 0 0 -100 0];
B = [0; 1; 0; 0; 0];
D = [-1 0  1  0  0]; % e = Dx (output)

% discretization
T = 0.1;
[Ad, Bd] = c2d(A, B, T)

% todo: fix hard-coded sizes
A1 = Ad(1:2,1:2);
B1 = Bd(1:2);
A3 = Ad(1:2,3:5);
A2 = Ad(3:5,3:5);
D1 = D(:,1:2);
D2 = D(:,3:5);

% controller stablizing pole-placement
ctrl1_F1 = -1*acker(A1, B1, [0 0])
ctrl2_F1 = -1*acker(A1, B1, [0.8+0.2*1i; 0.8-0.2*1i])

% solve for submatrices X and F2 for both controllers
% start by setting up kronecker product matrix for the controllers
kronmat = @(A1, B1, F1, A2, D1) ... % todo: fix hard-coded sizes
       [kron(eye(3), A1+B1*F1) - kron(A2', eye(2)), kron(eye(3), B1); ...
        kron(eye(3), D1),                           zeros(3,3)];
rhs_vec = @(A3, D2) -1*[A3(:); D2(:)];

ctrl1_v_X_F2 = kronmat(A1, B1, ctrl1_F1, A2, D1) \ rhs_vec(A3, D2);
ctrl1_X  = reshape(ctrl1_v_X_F2(1:6), [2,3]) % todo: fix hard-coded sizes
ctrl1_F2 = reshape(ctrl1_v_X_F2(7:9), [1,3]) % todo: fix hard-coded sizes

ctrl2_v_X_F2 = kronmat(A1, B1, ctrl2_F1, A2, D1) \ rhs_vec(A3, D2);
ctrl2_X  = reshape(ctrl1_v_X_F2(1:6), [2,3]) % todo: fix hard-coded sizes
ctrl2_F2 = reshape(ctrl1_v_X_F2(7:9), [1,3]) % todo: fix hard-coded sizes

% controller full-order observer pole-placement
ctrl1_L = -1*acker(Ad', D', [0 0 0 0 0])'
ctrl2_L = -1*acker(Ad', D', [0.1, 0.1+0.3*1i, 0.1-0.3*1i, 0.2+0.2*1i, 0.2-0.2*1i])'

% get controller tfs
ctrl1_F    = [ctrl1_F1 ctrl1_F2];
ctrl1_Acls = Ad + Bd*ctrl1_F + ctrl1_L*D;
ctrl2_F    = [ctrl2_F1 ctrl2_F2];
ctrl2_Acls = Ad + Bd*ctrl2_F + ctrl2_L*D;

[num_ctrl1, den_ctrl1] = ss2tf(ctrl1_Acls, -1*ctrl1_L, ctrl1_F, 0);
[num_ctrl2, den_ctrl2] = ss2tf(ctrl2_Acls, -1*ctrl2_L, ctrl2_F, 0);

ctrl1 = tf(num_ctrl1, den_ctrl1, T, 'Variable', 'z')
ctrl2 = tf(num_ctrl2, den_ctrl2, T, 'Variable', 'z')
