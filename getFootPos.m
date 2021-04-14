function footPos = getFeetPos(state)
    % Feet positions
    footPos = getFfoot(state);
    footPos = [footPos; getRfoot(state)];
end

function rear_foot = getRfoot(state)
    [x, y, theta, q1, q2, q3, q4] = deal(state(1), state(2), state(3), state(4), state(5), state(6), state(7));
    a = 0.15; b = 0.5; l = 0.2;
    cmb = [x; y];
    mid_lower_body_link = cmb + [a / 2 * sin(theta); -a / 2 * cos(theta)];
    % mid_upper_body_link = cmb + [-a / 2 * sin(theta); a / 2 * cos(theta)];
    joint3 = mid_lower_body_link + [-b / 2 * cos(theta); -b / 2 * sin(theta)];
    joint1 = mid_lower_body_link + [b / 2 * cos(theta); b / 2 * sin(theta)];
    cm3 = joint3 + [-l / 2 * cos(q3 + theta); -l / 2 * sin(q3 + theta)];
    joint4 = joint3 + [-l * cos(q3 + theta); -l * sin(q3 + theta)];

    rear_foot = joint4 + [-l * cos(theta + q3 + q4); -l * sin(theta + q3 + q4)];
end

function front_foot = getFfoot(state)
    [x, y, theta, q1, q2, q3, q4] = deal(state(1), state(2), state(3), state(4), state(5), state(6), state(7));
    a = 0.15; b = 0.5; l = 0.2;
    cmb = [x; y];
    mid_lower_body_link = cmb + [a / 2 * sin(theta); -a / 2 * cos(theta)];
    
    joint1 = mid_lower_body_link + [b / 2 * cos(theta); b / 2 * sin(theta)];
    joint2 = joint1 + [-l * cos(theta + q1); -l * sin(theta + q1)];

    front_foot = joint2 + [-l * cos(theta + q1 + q2); -l * sin(theta + q1 + q2)];
end