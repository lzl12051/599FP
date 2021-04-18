% clear;
% global h_ground tout yout err_flags t_end g;

% % The state after
% x_test = [-0.0029    0.2936    0.0011    0.7529    1.9691    0.7662    1.9675    0.0000    0.0000   -0.0011   -0.0003    0.0040    0.0013   -0.0004].';

% g = 9.81; t_end = 2;

% H = [Qlist, Rlist];
% params.H = diag(H)

% Q02 = [1000, 1000, 1000, 10, 10, 10, 10];
% R2 = [1e-6, 1e-6];
% R2list = [];
% Q2list = [];

% for i = 1:10
%     Q2list = [Q2list, Q02];
%     R2list = [R2list, R2];
% end

% H2 = [Q2list, R2list];
% params.H2 = diag(H2)

% ctrl = controller;
% ctrl.MpcFrontSwing(2, x_test)
clear;

l = 0.2; m = 0.25; mb = 8; a = 0.15; b = 0.5;
syms P1y Y_com X_com P1x I_b A B

I_bb = 1/12 * mb * (a^2 + b^2)
B_hat = [zeros(3, 2);
1 / mb, 0;
0, 1 / mb;
-(P1y - Y_com) / I_b, (P1x - X_com) / I_b;
0, 0]

A_mpc = ones(7,7)*A;
B_mpc = ones(7,2)*B;

Aeq = [eye(7), zeros(7, 7 * 9), -B_mpc, zeros(7, 2 * 9);
-A_mpc, eye(7), zeros(7, 7 * 8), zeros(7, 2), -B_mpc, zeros(7, 2 * 8);
zeros(7, 7), -A_mpc, eye(7), zeros(7, 7 * 7), zeros(7, 2 * 2), -B_mpc, zeros(7, 2 * 7);
zeros(7, 7 * 2), -A_mpc, eye(7), zeros(7, 7 * 6), zeros(7, 2 * 3), -B_mpc, zeros(7, 2 * 6);
zeros(7, 7 * 3), -A_mpc, eye(7), zeros(7, 7 * 5), zeros(7, 2 * 4), -B_mpc, zeros(7, 2 * 5);
zeros(7, 7 * 4), -A_mpc, eye(7), zeros(7, 7 * 4), zeros(7, 2 * 5), -B_mpc, zeros(7, 2 * 4);
zeros(7, 7 * 5), -A_mpc, eye(7), zeros(7, 7 * 3), zeros(7, 2 * 6), -B_mpc, zeros(7, 2 * 3);
zeros(7, 7 * 6), -A_mpc, eye(7), zeros(7, 7 * 2), zeros(7, 2 * 7), -B_mpc, zeros(7, 2 * 2);
zeros(7, 7 * 7), -A_mpc, eye(7), zeros(7, 7), zeros(7, 2 * 8), -B_mpc, zeros(7, 2);
zeros(7, 7 * 8), -A_mpc, eye(7), zeros(7, 2 * 9), -B_mpc]