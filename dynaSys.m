function funcs = dynaSys
    % This contains all dynamic systems
    funcs.fall = @fall;
    funcs.rearStance = @rearStance;
    funcs.doubleStanceQP = @doubleStanceQP;
end

function dx = fall(t, x)
    % Just falling down
    global g
    dx = [x(8:14); 0; -g; 0; 0; 0; 0; 0];
end

function dx = rearStance(t, x)
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

    Dq_ = [9, 0, sin(theta_1 + theta_2 + theta) / 40 + sin(theta_3 + theta_4 + theta) / 40 + (3 * sin(theta_1 + theta)) / 40 + (3 * sin(theta_3 + theta)) / 40 + (3 * cos(theta)) / 40, sin(theta_1 + theta_2 + theta) / 40 + (3 * sin(theta_1 + theta)) / 40, sin(theta_1 + theta_2 + theta) / 40, sin(theta_3 + theta_4 + theta) / 40 + (3 * sin(theta_3 + theta)) / 40, sin(theta_3 + theta_4 + theta) / 40; 0, 9, (3 * sin(theta)) / 40 - cos(theta_3 + theta_4 + theta) / 40 - (3 * cos(theta_1 + theta)) / 40 - (3 * cos(theta_3 + theta)) / 40 - cos(theta_1 + theta_2 + theta) / 40, -cos(theta_1 + theta_2 + theta) / 40 - (3 * cos(theta_1 + theta)) / 40, -cos(theta_1 + theta_2 + theta) / 40, -cos(theta_3 + theta_4 + theta) / 40 - (3 * cos(theta_3 + theta)) / 40, -cos(theta_3 + theta_4 + theta) / 40; sin(theta_1 + theta_2 + theta) / 40 + sin(theta_3 + theta_4 + theta) / 40 + (3 * sin(theta_1 + theta)) / 40 + (3 * sin(theta_3 + theta)) / 40 + (3 * cos(theta)) / 40, (3 * sin(theta)) / 40 - cos(theta_3 + theta_4 + theta) / 40 - (3 * cos(theta_1 + theta)) / 40 - (3 * cos(theta_3 + theta)) / 40 - cos(theta_1 + theta_2 + theta) / 40, cos(theta_2) / 100 + cos(theta_4) / 100 + (109^(1/2) * cos(theta_3 + theta_4 - atan(3/10))) / 800 - (3 * 109^(1/2) * cos(theta_1 + atan(3/10))) / 800 + (3 * 109^(1/2) * cos(theta_3 - atan(3/10))) / 800 - (109^(1/2) * cos(theta_1 + theta_2 + atan(3/10))) / 800 + 453/1600, cos(theta_2) / 100 - (3 * 109^(1/2) * cos(theta_1 + atan(3/10))) / 1600 - (109^(1/2) * cos(theta_1 + theta_2 + atan(3/10))) / 1600 + 1/60, cos(theta_2) / 200 - (109^(1/2) * cos(theta_1 + theta_2 + atan(3/10))) / 1600 + 1/300, cos(theta_4) / 100 + (109^(1/2) * cos(theta_3 + theta_4 - atan(3/10))) / 1600 + (3 * 109^(1/2) * cos(theta_3 - atan(3/10))) / 1600 + 1/60, cos(theta_4) / 200 + (109^(1/2) * cos(theta_3 + theta_4 - atan(3/10))) / 1600 + 1/300; sin(theta_1 + theta_2 + theta) / 40 + (3 * sin(theta_1 + theta)) / 40, -cos(theta_1 + theta_2 + theta) / 40 - (3 * cos(theta_1 + theta)) / 40, cos(theta_2) / 100 - (3 * 109^(1/2) * cos(theta_1 + atan(3/10))) / 1600 - (109^(1/2) * cos(theta_1 + theta_2 + atan(3/10))) / 1600 + 1/60, cos(theta_2) / 100 + 1/60, cos(theta_2) / 200 + 1/300, 0, 0; sin(theta_1 + theta_2 + theta) / 40, -cos(theta_1 + theta_2 + theta) / 40, cos(theta_2) / 200 - (109^(1/2) * cos(theta_1 + theta_2 + atan(3/10))) / 1600 + 1/300, cos(theta_2) / 200 + 1/300, 1/300, 0, 0; sin(theta_3 + theta_4 + theta) / 40 + (3 * sin(theta_3 + theta)) / 40, -cos(theta_3 + theta_4 + theta) / 40 - (3 * cos(theta_3 + theta)) / 40, cos(theta_4) / 100 + (109^(1/2) * cos(theta_3 + theta_4 - atan(3/10))) / 1600 + (3 * 109^(1/2) * cos(theta_3 - atan(3/10))) / 1600 + 1/60, 0, 0, cos(theta_4) / 100 + 1/60, cos(theta_4) / 200 + 1/300; sin(theta_3 + theta_4 + theta) / 40, -cos(theta_3 + theta_4 + theta) / 40, cos(theta_4) / 200 + (109^(1/2) * cos(theta_3 + theta_4 - atan(3/10))) / 1600 + 1/300, 0, 0, cos(theta_4) / 200 + 1/300, 1/300];
    Nqq_ = [(theta_dot^2 * cos(theta_1 + theta_2 + theta)) / 40 - (3 * theta_dot^2 * sin(theta)) / 40 + (theta_dot^2 * cos(theta_3 + theta_4 + theta)) / 40 + (theta_dot_1^2 * cos(theta_1 + theta_2 + theta)) / 40 + (theta_dot_2^2 * cos(theta_1 + theta_2 + theta)) / 40 + (theta_dot_3^2 * cos(theta_3 + theta_4 + theta)) / 40 + (theta_dot_4^2 * cos(theta_3 + theta_4 + theta)) / 40 + (3 * theta_dot^2 * cos(theta_1 + theta)) / 40 + (3 * theta_dot^2 * cos(theta_3 + theta)) / 40 + (3 * theta_dot_1^2 * cos(theta_1 + theta)) / 40 + (3 * theta_dot_3^2 * cos(theta_3 + theta)) / 40 + (theta_dot * theta_dot_1 * cos(theta_1 + theta_2 + theta)) / 20 + (theta_dot * theta_dot_2 * cos(theta_1 + theta_2 + theta)) / 20 + (theta_dot * theta_dot_3 * cos(theta_3 + theta_4 + theta)) / 20 + (theta_dot * theta_dot_4 * cos(theta_3 + theta_4 + theta)) / 20 + (theta_dot_1 * theta_dot_2 * cos(theta_1 + theta_2 + theta)) / 20 + (theta_dot_3 * theta_dot_4 * cos(theta_3 + theta_4 + theta)) / 20 + (3 * theta_dot * theta_dot_1 * cos(theta_1 + theta)) / 20 + (3 * theta_dot * theta_dot_3 * cos(theta_3 + theta)) / 20; (3 * theta_dot^2 * cos(theta)) / 40 + (theta_dot^2 * sin(theta_1 + theta_2 + theta)) / 40 + (theta_dot^2 * sin(theta_3 + theta_4 + theta)) / 40 + (theta_dot_1^2 * sin(theta_1 + theta_2 + theta)) / 40 + (theta_dot_2^2 * sin(theta_1 + theta_2 + theta)) / 40 + (theta_dot_3^2 * sin(theta_3 + theta_4 + theta)) / 40 + (theta_dot_4^2 * sin(theta_3 + theta_4 + theta)) / 40 + (3 * theta_dot^2 * sin(theta_1 + theta)) / 40 + (3 * theta_dot^2 * sin(theta_3 + theta)) / 40 + (3 * theta_dot_1^2 * sin(theta_1 + theta)) / 40 + (3 * theta_dot_3^2 * sin(theta_3 + theta)) / 40 + (3 * theta_dot * theta_dot_1 * sin(theta_1 + theta)) / 20 + (3 * theta_dot * theta_dot_3 * sin(theta_3 + theta)) / 20 + (theta_dot * theta_dot_1 * sin(theta_1 + theta_2 + theta)) / 20 + (theta_dot * theta_dot_2 * sin(theta_1 + theta_2 + theta)) / 20 + (theta_dot * theta_dot_3 * sin(theta_3 + theta_4 + theta)) / 20 + (theta_dot * theta_dot_4 * sin(theta_3 + theta_4 + theta)) / 20 + (theta_dot_1 * theta_dot_2 * sin(theta_1 + theta_2 + theta)) / 20 + (theta_dot_3 * theta_dot_4 * sin(theta_3 + theta_4 + theta)) / 20 + 8829/100; (2943 * sin(theta)) / 4000 - (981 * cos(theta_3 + theta_4 + theta)) / 4000 - (2943 * cos(theta_1 + theta)) / 4000 - (2943 * cos(theta_3 + theta)) / 4000 - (981 * cos(theta_1 + theta_2 + theta)) / 4000 + (9 * theta_dot_1^2 * cos(theta_1)) / 1600 + (9 * theta_dot_3^2 * cos(theta_3)) / 1600 + (3 * theta_dot_1^2 * sin(theta_1)) / 160 - (theta_dot_2^2 * sin(theta_2)) / 200 - (3 * theta_dot_3^2 * sin(theta_3)) / 160 - (theta_dot_4^2 * sin(theta_4)) / 200 + (3 * theta_dot_1^2 * cos(theta_1 + theta_2)) / 1600 + (3 * theta_dot_2^2 * cos(theta_1 + theta_2)) / 1600 + (3 * theta_dot_3^2 * cos(theta_3 + theta_4)) / 1600 + (3 * theta_dot_4^2 * cos(theta_3 + theta_4)) / 1600 + (theta_dot_1^2 * sin(theta_1 + theta_2)) / 160 + (theta_dot_2^2 * sin(theta_1 + theta_2)) / 160 - (theta_dot_3^2 * sin(theta_3 + theta_4)) / 160 - (theta_dot_4^2 * sin(theta_3 + theta_4)) / 160 + (theta_dot * theta_dot_1 * sin(theta_1 + theta_2)) / 80 + (theta_dot * theta_dot_2 * sin(theta_1 + theta_2)) / 80 - (theta_dot * theta_dot_3 * sin(theta_3 + theta_4)) / 80 - (theta_dot * theta_dot_4 * sin(theta_3 + theta_4)) / 80 + (theta_dot_1 * theta_dot_2 * sin(theta_1 + theta_2)) / 80 - (theta_dot_3 * theta_dot_4 * sin(theta_3 + theta_4)) / 80 + (9 * theta_dot * theta_dot_1 * cos(theta_1)) / 800 + (9 * theta_dot * theta_dot_3 * cos(theta_3)) / 800 + (3 * theta_dot * theta_dot_1 * sin(theta_1)) / 80 - (theta_dot * theta_dot_2 * sin(theta_2)) / 100 - (3 * theta_dot * theta_dot_3 * sin(theta_3)) / 80 - (theta_dot * theta_dot_4 * sin(theta_4)) / 100 - (theta_dot_1 * theta_dot_2 * sin(theta_2)) / 100 - (theta_dot_3 * theta_dot_4 * sin(theta_4)) / 100 + (3 * theta_dot * theta_dot_1 * cos(theta_1 + theta_2)) / 800 + (3 * theta_dot * theta_dot_2 * cos(theta_1 + theta_2)) / 800 + (3 * theta_dot * theta_dot_3 * cos(theta_3 + theta_4)) / 800 + (3 * theta_dot * theta_dot_4 * cos(theta_3 + theta_4)) / 800 + (3 * theta_dot_1 * theta_dot_2 * cos(theta_1 + theta_2)) / 800 + (3 * theta_dot_3 * theta_dot_4 * cos(theta_3 + theta_4)) / 800; -(981 * cos(theta_1 + theta_2 + theta)) / 4000 - (2943 * cos(theta_1 + theta)) / 4000 - (9 * theta_dot^2 * cos(theta_1)) / 1600 - (3 * theta_dot^2 * sin(theta_1)) / 160 - (theta_dot_2^2 * sin(theta_2)) / 200 - (3 * theta_dot^2 * cos(theta_1 + theta_2)) / 1600 - (theta_dot^2 * sin(theta_1 + theta_2)) / 160 - (theta_dot * theta_dot_2 * sin(theta_2)) / 100 - (theta_dot_1 * theta_dot_2 * sin(theta_2)) / 100; (theta_dot^2 * sin(theta_2)) / 200 - (981 * cos(theta_1 + theta_2 + theta)) / 4000 + (theta_dot_1^2 * sin(theta_2)) / 200 - (3 * theta_dot^2 * cos(theta_1 + theta_2)) / 1600 - (theta_dot^2 * sin(theta_1 + theta_2)) / 160 + (theta_dot * theta_dot_1 * sin(theta_2)) / 100; (3 * theta_dot^2 * sin(theta_3)) / 160 - (2943 * cos(theta_3 + theta)) / 4000 - (9 * theta_dot^2 * cos(theta_3)) / 1600 - (981 * cos(theta_3 + theta_4 + theta)) / 4000 - (theta_dot_4^2 * sin(theta_4)) / 200 - (3 * theta_dot^2 * cos(theta_3 + theta_4)) / 1600 + (theta_dot^2 * sin(theta_3 + theta_4)) / 160 - (theta_dot * theta_dot_4 * sin(theta_4)) / 100 - (theta_dot_3 * theta_dot_4 * sin(theta_4)) / 100; (theta_dot^2 * sin(theta_4)) / 200 - (981 * cos(theta_3 + theta_4 + theta)) / 4000 + (theta_dot_3^2 * sin(theta_4)) / 200 - (3 * theta_dot^2 * cos(theta_3 + theta_4)) / 1600 + (theta_dot^2 * sin(theta_3 + theta_4)) / 160 + (theta_dot * theta_dot_3 * sin(theta_4)) / 100];
    J_foot_back_ = [1, 0, sin(theta_3 + theta_4 + theta) / 5 + sin(theta_3 + theta) / 5 + (3 * cos(theta)) / 40 + sin(theta) / 4, 0, 0, sin(theta_3 + theta_4 + theta) / 5 + sin(theta_3 + theta) / 5, sin(theta_3 + theta_4 + theta) / 5; 0, 1, (3 * sin(theta)) / 40 - cos(theta_3 + theta) / 5 - cos(theta) / 4 - cos(theta_3 + theta_4 + theta) / 5, 0, 0, -cos(theta_3 + theta_4 + theta) / 5 - cos(theta_3 + theta) / 5, -cos(theta_3 + theta_4 + theta) / 5];
    dJ_foot_back_ = [0, 0, (cos(theta_3 + theta_4 + theta) * (theta_dot + theta_dot_3 + theta_dot_4)) / 5 + (cos(theta_3 + theta) * (theta_dot + theta_dot_3)) / 5 + (theta_dot * cos(theta)) / 4 - (3 * theta_dot * sin(theta)) / 40, 0, 0, (cos(theta_3 + theta_4 + theta) * (theta_dot + theta_dot_3 + theta_dot_4)) / 5 + (cos(theta_3 + theta) * (theta_dot + theta_dot_3)) / 5, (cos(theta_3 + theta_4 + theta) * (theta_dot + theta_dot_3 + theta_dot_4)) / 5; 0, 0, (sin(theta_3 + theta_4 + theta) * (theta_dot + theta_dot_3 + theta_dot_4)) / 5 + (sin(theta_3 + theta) * (theta_dot + theta_dot_3)) / 5 + (3 * theta_dot * cos(theta)) / 40 + (theta_dot * sin(theta)) / 4, 0, 0, (sin(theta_3 + theta_4 + theta) * (theta_dot + theta_dot_3 + theta_dot_4)) / 5 + (sin(theta_3 + theta) * (theta_dot + theta_dot_3)) / 5, (sin(theta_3 + theta_4 + theta) * (theta_dot + theta_dot_3 + theta_dot_4)) / 5];

    JcFc_ = double(J_foot_back_.' * (inv(J_foot_back_ * inv(Dq_) * J_foot_back_.') * (-dJ_foot_back_ * x(8:14) - J_foot_back_ * inv(Dq_) * (-Nqq_))));
    ddq = inv(Dq_) * (JcFc_ - Nqq_);

    dx = [x(8:14); ddq];
    % t
end

function dx = doubleStanceQP(t, x)
    eq = dynaEq;
    tau_all = QPCtrl(t, x);

    Dq_ = eq.Dq(x);
    Nqq_ = eq.Nqdq(x);

    J_foot_back_ = eq.jacRfoot(x);
    dJ_foot_back_ = eq.jacDotRfoot(x);

    J_foot_front_ = eq.jacFfoot(x);
    dJ_foot_front_ = eq.jacDotFfoot(x);

    Jc_all = [J_foot_back_; J_foot_front_];
    dJc_all = [dJ_foot_back_; dJ_foot_front_];

    Fc_ = inv(Jc_all * inv(Dq_) * Jc_all.') * (-dJc_all * x(8:14) - Jc_all * inv(Dq_) * (tau_all - Nqq_));
    JcFc_ = Jc_all.' * Fc_;

    ddq = inv(Dq_) * (tau_all + JcFc_ - Nqq_);
    dx = [x(8:14); ddq];
    % t_QP = t
end

function u = QPCtrl(t, x)
    l = 0.2; m = 0.25; g = 9.81; mb = 8; a = 0.15; b = 0.5;
    params.g = g; params.l = l; params.m = m; params.mb = mb; params.a = a; params.b = b;

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

    I_b = 1/12 * params.mb * (params.a^2 + params.b^2);
    bd = [params.mb * (Kpx * (0 - X_com) + Kdx * (0 - X_dot_com));
        params.mb * (Kpy * (0.3 - Y_com) + Kdy * (0 - Y_dot_com) + params.g);
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
