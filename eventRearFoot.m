function [value,isterminal,direction] = eventRearFoot(t,x)
    global h_ground %unit m
    foots = getFeetPos(x); %front back
    ground_ind = ceil(foots(3)/0.2);
    try 
        value = foots(4)-h_ground(ground_ind);
    catch
        value = foots(4);
    end
    isterminal = 1;   % stop the integration
    direction = 0;   % negative direction -1
end
