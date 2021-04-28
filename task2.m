clear;
global h_ground tout yout err_flags t_end g Q2
global params

% foot params here
params.Kpx = 1000;
params.Kdx = 1000;
params.Kpy = 1000;
params.Kdy = 1000;
params.foot_body = 4;
Q2 = [5000, 2000, 5, 50, 50, 5, 0];
params.dt = 0.04;
params.vx = 0.5;
params.step_time = 0.1;
% the step hight
params.high = 0.1;

g = 9.81; t_end = 5;
params.g = g
% Generate the ground hight data every 0.2m
% it's all 0 since we are running on a flat land
h_ground = zeros(1, ceil(30/0.2)); % unit: m

t_hip = pi / 3; t_knee = pi / 3;
t = 0; t1 = t_hip; t2 = t_knee; t3 = t_hip; t4 = t_knee;
l = 0.2; b = 0.5; a = 0.15;
Yb = l * sin(t + t3 + t4) + l * sin(t + t3) + b / 2 * sin(t) + a * cos(t) / 2;
ini_cond = [0.02; Yb; t; t1; t2; t3; t4; 0; 0; 0; 0; 0; 0; 0];
params.stand_h = Yb;

% dropAndBalance(ini_cond);

% TODO create a yout tout list (haven't test this)
yout = [ini_cond.']; % this is 1
tout = [0];

% If x coordinate of the body's CoM have not reach the goal
while yout(end, 1) <= 0.5
    % TODO Let the robot keep walking on the ground until x>30m
    rearStep();
    frontStep();
    
end

animate(tout, yout, 1/60, 'Videos/Task1_test.avi');
fprintf('Task 1 Score: %3.2f\n', calcScore);

function score = calcScore
    global tout
    total_time = tout(end);
    score = 100 / total_time;
end
