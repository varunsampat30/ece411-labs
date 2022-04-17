%% Define params
g = 9.81;
C = eye(4);
D = [0;0;0;0];
init_condit = [0;0;0;0];
multi_step = [1;0;0;0];

%% Sys params
M_A = 0.45;
m_A = 0.2;
l_A = 0.3;

M_B = 0.45;
m_B = 0.26; 
l_B = 0.35;

M_C = 0.45;
m_C = 0.3;
l_C = 0.4;

%% Define A, B matrix
[A_a, B_a] = pend_on_cart(M_A, m_A, l_A);
[A_b, B_b] = pend_on_cart(M_B, m_B, l_B);
[A_c, B_c] = pend_on_cart(M_C, m_C, l_C);

%%
%{
Smaller place values leads to slower but more controlled response
Larger place values leads to faster but less controlled response

After experimentation any pole in 0 -> -1 range is too slow
Anything beyond -3/-4 is too rapid 

Cannot take all poles to have the same value since place() command 
cannot place poles with multiplicity
greater than rank(B).
%}
F1 = -1*place(A_a, B_a, [-1.5 -1.6 -1.7 -1.8]);
F2 = -1*place(A_b, B_b, [-1.5 -1.6 -1.7 -1.8]);
F3 = -1*place(A_c, B_c, [-1.5 -1.6 -1.7 -1.8]);

%% 4.1 Setup args for TrueTime

pend_A_args = struct('A', A_a, 'B', B_a, 'F', F1, 'exec_time', 10e-3, 'period', 14e-3);
pend_B_args = struct('A', A_b, 'B', B_b, 'F', F2, 'exec_time', 14e-3, 'period', 20e-3);
pend_C_args = struct('A', A_c, 'B', B_c, 'F', F3, 'exec_time', 17e-3, 'period', 24e-3);

pends_args = struct( ...
    'schedule_algo', 1, ...
    'pend_data', [pend_A_args pend_B_args pend_C_args], ...
    'C', C, 'D', D, ...
    'start_index', 3, ...
    'end_index', 3);

%% 4.2.1 (rate monotonic)
pends_args = struct( ...
    'schedule_algo', 1, ...
    'pend_data', [pend_A_args pend_B_args pend_C_args], ...
    'C', C, 'D', D, ...
    'start_index', 1, ...
    'end_index', 3);

%% 4.2.2

pend_A_args = struct('A', A_a, 'B', B_a, 'F', F1, 'exec_time', 10e-3, 'period', 40e-3);
pend_B_args = struct('A', A_b, 'B', B_b, 'F', F2, 'exec_time', 14e-3, 'period', 56e-3);
pend_C_args = struct('A', A_c, 'B', B_c, 'F', F3, 'exec_time', 17e-3, 'period', 68e-3);

pends_args = struct( ...
    'schedule_algo', 1, ...
    'pend_data', [pend_A_args pend_B_args pend_C_args], ...
    'C', C, 'D', D, ...
    'start_index', 1, ...
    'end_index', 3);
%% 4.2.3
pend_A_args = struct('A', A_a, 'B', B_a, 'F', F1, 'exec_time', 10e-3, 'period', 50e-3);
pend_B_args = struct('A', A_b, 'B', B_b, 'F', F2, 'exec_time', 14e-3, 'period', 70e-3);
pend_C_args = struct('A', A_c, 'B', B_c, 'F', F3, 'exec_time', 17e-3, 'period', 85e-3);

pends_args = struct( ...
    'schedule_algo', 1, ...
    'pend_data', [pend_A_args pend_B_args pend_C_args], ...
    'C', C, 'D', D, ...
    'start_index', 1, ...
    'end_index', 3);

%% 4.2.4
pend_A_args = struct('A', A_a, 'B', B_a, 'F', F1, 'exec_time', 10e-3, 'period', 50e-3);
pend_B_args = struct('A', A_b, 'B', B_b, 'F', F2, 'exec_time', 14e-3, 'period', 70e-3);
pend_C_args = struct('A', A_c, 'B', B_c, 'F', F3, 'exec_time', 17e-3, 'period', 85e-3);

pends_args = struct( ...
    'schedule_algo', 2, ...
    'pend_data', [pend_A_args pend_B_args pend_C_args], ...
    'C', C, 'D', D, ...
    'start_index', 1, ...
    'end_index', 3);

%% 4.3
pend_A_args = struct('A', A_a, 'B', B_a, 'F', F1, 'exec_time', 10e-3, 'period', 15e-3);
pend_B_args = struct('A', A_b, 'B', B_b, 'F', F2, 'exec_time', 14e-3, 'period', 56e-3);
pend_C_args = struct('A', A_c, 'B', B_c, 'F', F3, 'exec_time', 17e-3, 'period', 68e-3);

pends_args = struct( ...
    'schedule_algo', 3, ...
    'pend_data', [pend_A_args pend_B_args pend_C_args], ...
    'C', C, 'D', D, ...
    'start_index', 1, ...
    'end_index', 3);


%% 4.4 (want to show prioFP fail @ 0.95

pend_A_args = struct('A', A_a, 'B', B_a, 'F', F1, 'exec_time', 10e-3, 'period', 22.2e-3);
pend_B_args = struct('A', A_b, 'B', B_b, 'F', F2, 'exec_time', 14e-3, 'period', 56e-3);
pend_C_args = struct('A', A_c, 'B', B_c, 'F', F3, 'exec_time', 17e-3, 'period', 68e-3);
pends_args = struct( ...
    'schedule_algo', 1, ...
    'pend_data', [pend_A_args pend_B_args pend_C_args], ...
    'C', C, 'D', D, ...
    'start_index', 1, ...
    'end_index', 3);

%% 4.4 Pass PrioEDF @ 0.95
pend_A_args = struct('A', A_a, 'B', B_a, 'F', F1, 'exec_time', 10e-3, 'period', 22.2e-3);
pend_B_args = struct('A', A_b, 'B', B_b, 'F', F2, 'exec_time', 14e-3, 'period', 56e-3);
pend_C_args = struct('A', A_c, 'B', B_c, 'F', F3, 'exec_time', 17e-3, 'period', 68e-3);

pends_args = struct( ...
    'schedule_algo', 3, ...
    'pend_data', [pend_A_args pend_B_args pend_C_args], ...
    'C', C, 'D', D, ...
    'start_index', 1, ...
    'end_index', 3);
