% The robot will drop down to the ground and keep balance
function dropAndBalance(ini_cond)
    global yout tout t_end
    impFuncs = impactMapping;
    dySys = dynaSys;
    % Setting up the initial conditions
    tout = 0;
    yout = ini_cond.';

    % Robot falling down
    ode_opt = odeset('Events', @eventRearTouchGround);
    [t, y, te, ye, ie] = ode45(@dySys.fall, [0 t_end], ini_cond, ode_opt);
    nt = length(t);
    t_start = t(nt);
    tout = [tout; t(2:nt)];
    yout = [yout; y(2:nt, :)];
    % Mapping rear leg's impact
    impFuncs.rearMapping();

    % Rear stance
    ode_opt = odeset('Events', @eventFrontTouchGround);
    [t, y, te, ye, ie] = ode45(@dySys.rearStance, [t_start t_end], yout(end, :), ode_opt);
    nt = length(t);
    t_start = t(nt);
    tout = [tout; t(2:nt)];
    yout = [yout; y(2:nt, :)];
    % Mapping front leg's impact
    impFuncs.frontMapping();

    % Double stance with QP controller to balance the robot
    ode_opt = odeset('Events', @eventDoubleStaceQP);
    [t, y, te, ye, ie] = ode45(@dySys.doubleStanceQP, [t_start t_end], yout(end, :), ode_opt);
    nt = length(t);
    tout = [tout; t(2:nt)];
    yout = [yout; y(2:nt, :)];
    tstart = t(nt);

end

function [value, isterminal, direction] = eventRearTouchGround(~, x)
    feet_pos = getFeetPos(x);
    value = feet_pos(4); % detect height = 0
    isterminal = 1; % stop the integration
    direction = -1; % negative direction
end

function [value, isterminal, direction] = eventFrontTouchGround(~, x)
    feet_pos = getFeetPos(x);
    value = feet_pos(2); %double(subs(foot_front(2))); % detect height = 0
    isterminal = 1; % stop the integration
    direction = -1; % negative direction
end

function [value, isterminal, direction] = eventDoubleStaceQP(t, x)
    % XXX should not use time to decide when to stop ODE
    value = 2 - t; % detect height = 0
    isterminal = 1; % stop the integration
    direction = -1; % negative direction
end
