function funcs = controller
    funcs.QpBalance = @QpBalance;
    funcs.MpcFrontSwing = @MpcFrontSwing;
end

function u = QpBalance(t, x)
    l = 0.2; m = 0.25; g = 9.81; mb = 8; a = 0.15; b = 0.5;

    X_com = x(1);
    Y_com = x(2);
    theta = x(3);
    theta_1 = x(4);
    theta_2 = x(5);
    theta_3 = x(6);
    theta_4 = x(7);
    X_dot_com = x(8);
    Y_dot_com = x(9);
    theta_dot = x(10);
    theta_dot_1 = x(11);
    theta_dot_2 = x(12);
    theta_dot_3 = x(13);
    theta_dot_4 = x(14);

    % front
    P1x = X_com - cos(theta_1 + theta_2 + theta) / 5 - cos(theta_1 + theta) / 5 + cos(theta) / 4 + (3 * sin(theta)) / 40;
    P1y = Y_com - sin(theta_1 + theta_2 + theta) / 5 - sin(theta_1 + theta) / 5 - (3 * cos(theta)) / 40 + sin(theta) / 4;
    % back
    P2x = X_com - cos(theta_3 + theta_4 + theta) / 5 - cos(theta_3 + theta) / 5 - cos(theta) / 4 + (3 * sin(theta)) / 40;
    P2y = Y_com - sin(theta_3 + theta_4 + theta) / 5 - sin(theta_3 + theta) / 5 - (3 * cos(theta)) / 40 - sin(theta) / 4;

    alpha = 1;
    A = [1, 0, 1, 0; 0, 1, 0, 1; -(P1y - Y_com), (P1x - X_com), -(P2y - Y_com), (P2x - X_com)];
    S = diag([10, 10, 100]);

    Kpx = 200;
    Kdx = 10;
    Kpy = 200;
    Kdy = 50;
    Kpt = 300;
    Kdt = 10;

    I_b = 1/12 * mb * (a^2 + b^2);
    bd = [mb * (Kpx * (0 - X_com) + Kdx * (0 - X_dot_com));
        mb * (Kpy * (0.3 - Y_com) + Kdy * (0 - Y_dot_com) + g);
        I_b * (Kpt * (0 - theta) + Kdt * (0 - theta_dot))];

    A_qp = [0, 1, 0, 0;
        0, 0, 0, 1;
        0, -1, 0, 0;
        0, 0, 0, -1;
        1, -0.5, 0, 0;
        -1, -0.5, 0, 0;
        0, 0, 1, -0.5;
        0, 0, -1, -0.5];

    B_qp = [500; 500; -10; -10; 0; 0; 0; 0];
    H = alpha * eye(4) + A' * S * A;
    f = -bd' * S * A;
    qp_opt = optimset('Display', 'none');
    u_ = quadprog(H, f, A_qp, B_qp, [], [], [], [], [], qp_opt);
    R = double([cos(theta), -sin(theta); sin(theta), cos(theta)]);
    Fc_front = [u_(1); u_(2)];
    Fc_back = [u_(3); u_(4)];
    Jc_front = [sin(theta_1 + theta_2) / 5 + sin(theta_1) / 5, sin(theta_1 + theta_2) / 5; -cos(theta_1 + theta_2) / 5 - cos(theta_1) / 5, -cos(theta_1 + theta_2) / 5];
    Jc_back = [sin(theta_3 + theta_4) / 5 + sin(theta_3) / 5, sin(theta_3 + theta_4) / 5; -cos(theta_3 + theta_4) / 5 - cos(theta_3) / 5, -cos(theta_3 + theta_4) / 5];

    tau_f = -Jc_front' * R' * Fc_front;
    tau_b = -Jc_back' * R' * Fc_back;
    % tau_all = [0;0;0;tau_f;tau_b];
    u = [0; 0; 0; tau_f; tau_b];

end

function u = MpcFrontSwing(t, x)
    global g H2 A_c2 B_c2 Kpx Kpy Kdx Kdy
    l = 0.2; m = 0.25; mb = 8; a = 0.15; b = 0.5;

    X_com = x(1); Y_com = x(2); theta = x(3); theta_1 = x(4); theta_2 = x(5); theta_3 = x(6); theta_4 = x(7);
    X_dot_com = x(8); Y_dot_com = x(9); theta_dot = x(10); theta_dot_1 = x(11); theta_dot_2 = x(12); theta_dot_3 = x(13); theta_dot_4 = x(14);

    feet_pos = getFeetPos(x);
    % front foot
    P1x = feet_pos(1); %X_com - cos(theta_1 + theta_2 + theta) / 5 - cos(theta_1 + theta) / 5 + cos(theta) / 4 + (3 * sin(theta)) / 40;
    P1y = feet_pos(2); %Y_com - sin(theta_1 + theta_2 + theta) / 5 - sin(theta_1 + theta) / 5 - (3 * cos(theta)) / 40 + sin(theta) / 4;
    % rear foot
    P2x = feet_pos(3); %X_com - cos(theta_3 + theta_4 + theta) / 5 - cos(theta_3 + theta) / 5 - cos(theta) / 4 + (3 * sin(theta)) / 40;
    P2y = feet_pos(4); %Y_com - sin(theta_3 + theta_4 + theta) / 5 - sin(theta_3 + theta) / 5 - (3 * cos(theta)) / 40 - sin(theta) / 4;

    I_b = 1/12 * mb * (a^2 + b^2); % I_b = 0.1817

    % control the swing of front foot
    % XXX change t_now -> t_swing_start
    global t_swing_start
    % desired x position and x velocity
    % XXX X position control not implemented

    % BUG maybe xfd = 0; dxfd = P1x;
    xfd = P1x; dxfd = 0; % P1x + 0.1;
    % calculate the desired y position and y velocity
    if t - t_swing_start < 0.1
        dyfd = 0.1/0.1;
        yfd = (t - t_swing_start) * dyfd;
    else
        dyfd = -0.1/0.1;
        yfd = 0.1 + (t - t_swing_start - 0.1) * dyfd;
    end

    % front v
    % XXX move this to dynaEq
    P1xv = X_dot_com + (theta_dot_1 + theta_dot_2 + theta_dot) * sin(theta_1 + theta_2 + theta) / 5 + (theta_dot_1 + theta_dot) * sin(theta_1 + theta) / 5 - theta_dot * sin(theta) / 4 + (3 * theta_dot * cos(theta)) / 40;
    P1yv = Y_dot_com - (theta_dot_1 + theta_dot_2 + theta_dot) * cos(theta_1 + theta_2 + theta) / 5 - (theta_dot_1 + theta_dot) * cos(theta_1 + theta) / 5 + (3 * theta_dot * sin(theta)) / 40 + theta_dot * cos(theta) / 4;

    % front foot PD controller
    uf1 = Kpx * (xfd - P1x) + Kdx * (dxfd - P1xv);
    uf2 = Kpy * (yfd - P1y) + Kdy * (dyfd - P1yv);
    u_front = [uf1, uf2];

    % MPC for balance the body by rear foot
    % Lecture 14 A^hat
    % 0     0     0     1     0     0     0
    % 0     0     0     0     1     0     0
    % 0     0     0     0     0     1     0
    % 0     0     0     0     0     0     0
    % 0     0     0     0     0     0    -1
    % 0     0     0     0     0     0     0
    % 0     0     0     0     0     0     0
    A_hat = [zeros(3, 3), eye(3), zeros(3, 1); zeros(4, 7)];
    A_hat(4, 7) = -u_front(1) / (mb * g);
    A_hat(5, 7) = -u_front(2) / (mb * g) - 1;
    A_hat(6, 7) = u_front(1) * (P1y - Y_com) / I_b - u_front(2) * (P1x - X_com) / I_b;

    % B^hat
    % [                 0,                 0]
    % [                 0,                 0]
    % [                 0,                 0]
    % [               1/8,                 0]
    % [                 0,               1/8]
    % [-(P2y - Y_com)/I_b, (P2x - X_com)/I_b]
    % [                 0,                 0]

    B_hat = [zeros(3, 2);
        1 / mb, 0;
        0, 1 / mb;
        -(P2y - Y_com) / I_b, (P2x - X_com) / I_b;
        0, 0];

    % u
    % [F2x; F2y]

    dt = 0.04;
    A_mpc = A_hat * dt + eye(7);
    B_mpc = B_hat * dt;

    % Aeq checked 70x90
    Aeq = [eye(7), zeros(7, 7 * 9), -B_mpc, zeros(7, 2 * 9);
        -A_mpc, eye(7), zeros(7, 7 * 8), zeros(7, 2), -B_mpc, zeros(7, 2 * 8);
        zeros(7, 7), -A_mpc, eye(7), zeros(7, 7 * 7), zeros(7, 2 * 2), -B_mpc, zeros(7, 2 * 7);
        zeros(7, 7 * 2), -A_mpc, eye(7), zeros(7, 7 * 6), zeros(7, 2 * 3), -B_mpc, zeros(7, 2 * 6);
        zeros(7, 7 * 3), -A_mpc, eye(7), zeros(7, 7 * 5), zeros(7, 2 * 4), -B_mpc, zeros(7, 2 * 5);
        zeros(7, 7 * 4), -A_mpc, eye(7), zeros(7, 7 * 4), zeros(7, 2 * 5), -B_mpc, zeros(7, 2 * 4);
        zeros(7, 7 * 5), -A_mpc, eye(7), zeros(7, 7 * 3), zeros(7, 2 * 6), -B_mpc, zeros(7, 2 * 3);
        zeros(7, 7 * 6), -A_mpc, eye(7), zeros(7, 7 * 2), zeros(7, 2 * 7), -B_mpc, zeros(7, 2 * 2);
        zeros(7, 7 * 7), -A_mpc, eye(7), zeros(7, 7), zeros(7, 2 * 8), -B_mpc, zeros(7, 2);
        zeros(7, 7 * 8), -A_mpc, eye(7), zeros(7, 2 * 9), -B_mpc];

    xk = [X_com; Y_com - 0.3; theta; X_dot_com; Y_dot_com; theta_dot; g];
    % XXX check xk or x
    beq = [A_mpc * xk; zeros(7 * 9, 1)];
    % f = zeros(110,1);
    f2 = zeros(70 + 2 * 10, 1);
    % u_ = quadprog(H,f,A_c2,B_c2,Aeq,beq);
    u_ = quadprog(H2, f2, A_c2, B_c2, Aeq, beq);

    % Rotation matrix world->body
    theta = x(3);
    R = double([cos(theta), -sin(theta); sin(theta), cos(theta)]);
    Fc_front = [u_front(1); u_front(2)]
    % XXX using magic here
    Fc_back = [u_(71); u_(72)] % [70;90]; %

    % Jacobian from hip to foot
    % NOTE jac(hip->foot);
    Jc_front = [sin(theta_1 + theta_2) / 5 + sin(theta_1) / 5, sin(theta_1 + theta_2) / 5; -cos(theta_1 + theta_2) / 5 - cos(theta_1) / 5, -cos(theta_1 + theta_2) / 5];
    Jc_back = [sin(theta_3 + theta_4) / 5 + sin(theta_3) / 5, sin(theta_3 + theta_4) / 5; -cos(theta_3 + theta_4) / 5 - cos(theta_3) / 5, -cos(theta_3 + theta_4) / 5];

    tau_f = Jc_front' * R' * Fc_front;
    tau_b = -Jc_back' * R' * Fc_back;
    u = [0; 0; 0; tau_f; tau_b];

end
