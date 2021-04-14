function [value,isterminal,direction] = eventFrontFoot(t,x)
    global h_ground %unit m
    foots = getFeetPos(x) %front back
    ground_ind = ceil(foots(1)/0.2)
    try 
        value = foots(2)-h_ground(ground_ind)
    catch
        value = foots(2)
    end
    isterminal = 1;   % stop the integration
    direction = 0;   % negative direction -1
end
