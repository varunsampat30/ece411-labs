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
ctrl2_X  = reshape(ctrl2_v_X_F2(1:6), [2,3]) % todo: fix hard-coded sizes
ctrl2_F2 = reshape(ctrl2_v_X_F2(7:9), [1,3]) % todo: fix hard-coded sizes

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

%% In lab
K = 1e-5;
Pz = tf([1], [1 -1], T);
Pz_num = cell2mat(Pz.Numerator);
Pz_den = cell2mat(Pz.Denominator);

z = tf('z', T);
Cz = ((z-0.522)*(z+0.45))/((z - 0.96)*(z-0.94)*(z-0.92));
Cz_num = cell2mat(Cz.Numerator);
Cz_den = cell2mat(Cz.Denominator);

CLSz = feedback(K*Cz, Pz)
p = pole(CLSz);

%% quantize maal
q = 2^(-16);
K_quant = quant(K, q);
Cz_num_quant = quant(Cz_num, q);
Cz_den_quant = quant(Cz_den, q);
Cz_quant = tf(Cz_num_quant, Cz_den_quant, T)

CLS_quant = feedback(K_quant*Cz_quant, Pz)
p_quant = pole(CLS_quant)

%% Decomposing controllers
C1_z = tf(1, [1 -0.96], T);
C2_z = tf([1 -0.522], [1 -0.94], T);
C3_z = tf([1 0.45], [1 -0.92], T);

C1_z_num = cell2mat(C1_z.Numerator);
C1_z_den = cell2mat(C1_z.Denominator);
C1_z_num_quant = quant(C1_z_num, q);
C1_z_den_quant = quant(C1_z_den, q);
C1_quant = tf(C1_z_num_quant, C1_z_den_quant, T);

C2_z_num = cell2mat(C2_z.Numerator);
C2_z_den = cell2mat(C2_z.Denominator);
C2_z_num_quant = quant(C2_z_num, q);
C2_z_den_quant = quant(C2_z_den, q);
C2_quant = tf(C2_z_num_quant, C2_z_den_quant, T);

C3_z_num = cell2mat(C3_z.Numerator);
C3_z_den = cell2mat(C3_z.Denominator);
C3_z_num_quant = quant(C3_z_num, q);
C3_z_den_quant = quant(C3_z_den, q);
C3_quant = tf(C3_z_num_quant, C3_z_den_quant, T);

C_eff = C1_quant * C2_quant * C3_quant;

CLS_quant_2 = feedback(K_quant*C_eff, Pz);
p_quant_eff = pole(CLS_quant_2);

%% Section 4.3
Pz_new = 0.25/((z - 1)*(z - 0.5));
Pz_new_num = cell2mat(Pz_new.Numerator);
Pz_new_den = cell2mat(Pz_new.Denominator);

%% Section 4.4
q = 2^(-16);
[ctrl1_zeros, ctrl1_poles, ctrl1_gain] = tf2zpk( ...
    cell2mat(ctrl1.Numerator), ...
    cell2mat(ctrl1.Denominator) ...
    );

ctrl1_z_quant = quant(ctrl1_zeros', q);
ctrl1_p_quant = quant(ctrl1_poles', q);
ctrl1_k_quant = quant(ctrl1_gain, q);

ctrl1_quant = tf(zpk(ctrl1_z_quant, ctrl1_p_quant, ctrl1_k_quant, T));
ctrl1_quant_num = cell2mat(ctrl1_quant.numerator);
ctrl1_quant_den = cell2mat(ctrl1_quant.denominator);

[ctrl2_zeros, ctrl2_poles, ctrl2_gain] = tf2zpk( ...
    cell2mat(ctrl2.Numerator), ...
    cell2mat(ctrl2.Denominator) ...
    );

ctrl2_z_quant = quant(ctrl2_zeros', q);
ctrl2_p_quant = quant(ctrl2_poles', q);
ctrl2_k_quant = quant(ctrl2_gain, q);

ctrl2_quant = tf(zpk(ctrl2_z_quant, ctrl2_p_quant, ctrl2_k_quant, T));
ctrl2_quant_num = cell2mat(ctrl2_quant.numerator);
ctrl2_quant_den = cell2mat(ctrl2_quant.denominator);
    
% plant_sys = ss(A, B, D, 0);
plant_sys = tf(1, [1 0 1]);




