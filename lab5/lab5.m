%% Define params
g = 9.81;
C = eye(4);
D = [0;0;0;0];
init_condit = [0;0;0;0];

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
[A_b, B_b] = pend_on_cart(M_A, m_A, l_A);
[A_c, B_c] = pend_on_cart(M_A, m_A, l_A);

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

% args = [F...]