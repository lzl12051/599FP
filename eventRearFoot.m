function [value, isterminal, direction] = eventRearFootGround(~, x)
    global h_ground %unit m
    feet_pos = getFeetPos(x); %front back
    ground_index = ceil(feet_pos(3) / 0.2);

    try
        value = feet_pos(4) - h_ground(ground_index);
    catch
        value = feet_pos(4);
    end

    isterminal = 1; % stop the integration
    direction = 0; % negative direction -1
end

% small test case

% global h_ground
% h_ground = [0.1,0.2,0.5,0.5,0.6]
% t = 0.1
% x = [0;0.4;0.1;pi/4;pi/2;pi/4;pi/2]
% eventRearFootGround(t,x)
