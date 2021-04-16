global h_ground tout yout err_flags

% Generate the ground hight data every 0.2m
% it's all 0 since we are running on a flat land
h_ground = zeros(1, ceil(30/0.2)); % unit: m

while yout(end, 1) <= 30:
    % TODO Let the robot keep walking on the ground until x>30m
    frontSwing();
    rearSwing();
end

fprintf('Task 1 Score: %3.2f\n', calcScore);

function score = calcScore
    global tout
    total_time = tout(end);
    score = 100 / total_time;
end