clear;
global h_ground tout yout err_flags t_end g;

g = 9.81; t_end = 5;
% Generate the ground hight data every 0.2m
% it's all 0 since we are running on a flat land
h_ground = zeros(1, ceil(30/0.2)); % unit: m

t = 0.1; t1 = pi / 4; t2 = pi / 2; t3 = pi / 4; t4 = pi / 2;
ini_cond = [0; 0.5; t; t1; t2; t3; t4; 0; 0; 0; 0; 0; 0; 0];

dropAndBalance(ini_cond);
frontStep();
% % If x coordinate of the body's CoM have not reach the goal
% while yout(end, 1) <= 30
%     % TODO Let the robot keep walking on the ground until x>30m

%     frontStep();
%     rearStep();
% end

animate(tout, yout, 1/60, 'Videos/Task1_test.avi');
fprintf('Task 1 Score: %3.2f\n', calcScore);

function score = calcScore
    global tout
    total_time = tout(end);
    score = 100 / total_time;
end
