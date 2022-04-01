function [A, B] = pend_on_cart(M, m, l)
g = 9.81;
A = [0 1 0 0; 0 0 -m*g/M 0; 0 0 0 1; 0 0 (M+m)*g/(M*l) 0];
B = [0; 1/M; 0; -1/(M*l)];
end