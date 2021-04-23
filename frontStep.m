% The robot will swing its front foot
function frontStep
    global yout tout H2 t_end A_c2 B_c2 Kpx Kpy Kdx Kdy t_swing_start
    dySys = dynaSys;
    % FIXME here needs some work
    Q2 = [5000, 5000, 3000, 500, 500, 1, 0];
    R2 = [1e-10, 1e-10];
    R2list = repmat(R2, 1, 10);
    Q2list = repmat(Q2, 1, 10);

    H2 = [Q2list, R2list];
    H2 = diag(H2);

    A_c2 = [];
    B_c2 = [];

    for i = 1:10
        A_c2 = [A_c2; zeros(1, 70), zeros(1, 2 * (i - 1)), 0, 1, zeros(1, 2 * (9 - i + 1));
            zeros(1, 70), zeros(1, 2 * (i - 1)), 0, -1, zeros(1, 2 * (9 - i + 1));
            zeros(1, 70), zeros(1, 2 * (i - 1)), 1, -0.5, zeros(1, 2 * (9 - i + 1));
            zeros(1, 70), zeros(1, 2 * (i - 1)), -1, -0.5, zeros(1, 2 * (9 - i + 1)); ];
        B_c2 = [B_c2; 500; -10; 0; 0];
    end

    Kpx = 1;
    Kdx = 1;
    Kpy = 4;
    Kdy = 4;

    t_start = tout(end)
    t_swing_start = t_start;
    options = odeset('Events', @eventFrontTouchGround);
    [t, y, te, ye, ie] = ode45(@dySys.FrontSwingMpc, [t_start t_end], yout(end, :), options);
    nt = length(t);
    tout = [tout; t(2:nt)];
    yout = [yout; y(2:nt, :)];
    tstart = t(nt);

    % % keep balance after step

    % impMapping = impactMapping;
    % impMapping.frontMapping();

    % % Double stance with QP controller to balance the robot
    % ode_opt = odeset('Events', @eventDoubleStaceQP);
    % [t, y, te, ye, ie] = ode45(@dySys.doubleStanceQP, [t_start t_end], yout(end, :), ode_opt);
    % nt = length(t);
    % tout = [tout; t(2:nt)];
    % yout = [yout; y(2:nt, :)];
    % %tstart = t(nt);

end

function [value, isterminal, direction] = eventFrontTouchGround(~, x)
    feet_pos = getFeetPos(x);
    value = feet_pos(2); %double(subs(foot_front(2))); % detect height = 0
    isterminal = 1; % stop the integration
    direction = -1; % negative direction
end

function [value, isterminal, direction] = eventDoubleStaceQP(t, x)
    global t_end
    % XXX should not use time to decide when to stop ODE
    value = abs(x(9)) > 0.01; % detect height = 0
    isterminal = 1; % stop the integration
    direction = -1; % negative direction
end
