function funcs = dynaEq
    % this will contains all matrices we might need
    % add functions or matrices here for convenience
    % XXX I haven't checked if they are all correct, no guarantee here for the correctness

    % Jacobian family
    funcs.jacFfoot = @jacFfoot;
    funcs.jacRfoot = @jacRfoot;
    funcs.jacDotRfoot = @jacDotRfoot;
    funcs.jacDotFfoot = @jacDotFfoot;

    % some other useful matrix
    funcs.Dq = @Dq;
    funcs.iDq = @iDq;
    funcs.Nqdq = @Nqdq;
end

function jac_rear_foot = jacRfoot(state)
    % get the numerical value of states
    [theta, q3, q4] = deal(state(3), state(6), state(7));
    % compute the numerical jacobian matrix of rear foot
    jac_rear_foot = [1, 0, sin(q3 + q4 + theta) / 5 + sin(q3 + theta) / 5 + (3 * cos(theta)) / 40 + sin(theta) / 4, 0, 0, sin(q3 + q4 + theta) / 5 + sin(q3 + theta) / 5, sin(q3 + q4 + theta) / 5; 0, 1, (3 * sin(theta)) / 40 - cos(q3 + theta) / 5 - cos(theta) / 4 - cos(q3 + q4 + theta) / 5, 0, 0, - cos(q3 + q4 + theta) / 5 - cos(q3 + theta) / 5, -cos(q3 + q4 + theta) / 5];
end

function jac_front_foot = jacFfoot(state)
    % get the numerical value of states
    % [theta, q1, q2] = deal( state(3), state(4), state(5));
    % compute the numerical jacobian matrix of front foot
    % jac_front_foot = [1, 0, (3 * cos(theta)) / 40 - sin(theta) / 4 + sin(q1 + q2 + theta) / 5 + sin(q1 + theta) / 5, sin(q1 + q2 + theta) / 5 + sin(q1 + theta) / 5, sin(q1 + q2 + theta) / 5, 0, 0; 0, 1, cos(theta) / 4 + (3 * sin(theta)) / 40 - cos(q1 + q2 + theta) / 5 - cos(q1 + theta) / 5, - cos(q1 + q2 + theta) / 5 - cos(q1 + theta) / 5, -cos(q1 + q2 + theta) / 5, 0, 0];
    [theta, theta_1, theta_2] = deal( state(3), state(4), state(5));
    jac_front_foot = [1,0,sin(theta_1+theta_2+theta)/5+sin(theta_1+theta)/5+(3*cos(theta))/40-sin(theta)/4,sin(theta_1+theta_2+theta)/5+sin(theta_1+theta)/5,sin(theta_1+theta_2+theta)/5,0,0;0,1,cos(theta)/4-cos(theta_1+theta)/5-cos(theta_1+theta_2+theta)/5+(3*sin(theta))/40,-cos(theta_1+theta_2+theta)/5-cos(theta_1+theta)/5,-cos(theta_1+theta_2+theta)/5,0,0];
end

function jc_dot_rear_foot = jacDotRfoot(state)
    % get the numerical value of states
    [theta,q3, q4] = deal(state(3), state(6), state(7));
    [theta_dot, q_dot_3, q_dot_4] = deal(state(10), state(13), state(14));
    % compute the numerical jacobian matrix dot of rear foot
    jc_dot_rear_foot = [0, 0, (q_dot_3 + q_dot_4 + theta_dot) * cos(q3 +q4 + theta) / 5 + (q_dot_3 + theta_dot) * cos(q3 + theta) / 5 - (3 * theta_dot * sin(theta)) / 40 + theta_dot * cos(theta) / 4, 0, 0, (q_dot_3 + q_dot_4 + theta_dot) * cos(q3 + q4 + theta) / 5 + (q_dot_3 + theta_dot) * cos(q3 + theta) / 5, (q_dot_3 + q_dot_4 + theta_dot) * cos(q3 + q4 + theta) / 5;
                    0, 0, (3 * theta_dot * cos(theta)) / 40 + (q_dot_3 + theta_dot) * sin(q3 + theta) / 5 + theta_dot * sin(theta) / 4 + (q_dot_3 + q_dot_4 + theta_dot) * sin(q3 + q4 + theta) / 5, 0, 0, (q_dot_3 + q_dot_4 + theta_dot) * sin(q3 + q4 + theta) / 5 + (q_dot_3 +theta_dot) * sin(q3 + theta) / 5, (q_dot_3 + q_dot_4 + theta_dot) * sin(q3 + q4 + theta) / 5];
end

function jc_dot_front_foot = jacDotFfoot(state)
    % get the numerical value of states
    % [theta, q1, q2] = deal(state(3), state(4), state(5));
    % [theta_dot, q_dot_1, q_dot_2] = deal(state(10), state(11), state(12));
    % compute the numerical jacobian matrix dot of front foot
    % jc_dot_front_foot = [0, 0, (cos(q1 + theta) * (q_dot_1 + theta_dot)) / 5 - (3 * theta_dot * sin(theta)) / 40 - (theta_dot * cos(theta)) / 4 + (cos(q1 + q2 + theta) * (q_dot_1 + q_dot_2 + theta_dot)) / 5, (cos(q1 + theta) * (q_dot_1 + theta_dot)) / 5 + (cos(q1 + q2 + theta) * (q_dot_1 + q_dot_2 + theta_dot)) / 5, (cos(q1 + q2 + theta) * (q_dot_1 + q_dot_2 + theta_dot)) / 5, 0, 0;
    %                 0, 0, (3 * theta_dot * cos(theta)) / 40 - (theta_dot * sin(theta)) / 4 + (sin(q1 + theta) * (q_dot_1 + theta_dot)) / 5 + (sin(q1 + q2 + theta) * (q_dot_1 + q_dot_2 + theta_dot)) / 5, (sin(q1 + theta) * (q_dot_1 + theta_dot)) / 5 + (sin(q1 + q2 + theta) * (q_dot_1 + q_dot_2 + theta_dot)) / 5, (sin(q1 + q2 + theta) * (q_dot_1 + q_dot_2 + theta_dot)) / 5, 0, 0];
    [theta, theta_1, theta_2] = deal(state(3), state(4), state(5));
    [theta_dot, theta_dot_1, theta_dot_2] = deal(state(10), state(11), state(12));
    jc_dot_front_foot = [0,0,(cos(theta_1+theta_2+theta)*(theta_dot+theta_dot_1+theta_dot_2))/5+(cos(theta_1+theta)*(theta_dot+theta_dot_1))/5-(theta_dot*cos(theta))/4-(3*theta_dot*sin(theta))/40,(cos(theta_1+theta_2+theta)*(theta_dot+theta_dot_1+theta_dot_2))/5+(cos(theta_1+theta)*(theta_dot+theta_dot_1))/5,(cos(theta_1+theta_2+theta)*(theta_dot+theta_dot_1+theta_dot_2))/5,0,0;0,0,(sin(theta_1+theta_2+theta)*(theta_dot+theta_dot_1+theta_dot_2))/5+(sin(theta_1+theta)*(theta_dot+theta_dot_1))/5+(3*theta_dot*cos(theta))/40-(theta_dot*sin(theta))/4,(sin(theta_1+theta_2+theta)*(theta_dot+theta_dot_1+theta_dot_2))/5+(sin(theta_1+theta)*(theta_dot+theta_dot_1))/5,(sin(theta_1+theta_2+theta)*(theta_dot+theta_dot_1+theta_dot_2))/5,0,0];
end

function Dq_matrix = Dq(state)
    % get the numerical value of states
    [theta, q1, q2, q3, q4] = deal(state(3), state(4), state(5), state(6), state(7));
    % compute the numerical mass matrix of the robot
    Dq_matrix = [(9), (0), sin(q1 + q2 + theta) / 40 + sin(q3 + q4 + theta) / 40 + (3 * sin(q1 + theta)) / 40 + (3 * sin(q3 + theta)) / 40 + ((3) * cos(theta)) / 40, sin(q1 + q2 + theta) / 40 + (3 * sin(q1 + theta)) / 40, sin(q1 + q2 + theta) / 40, sin(q3 + q4 + theta) / 40 + (3 * sin(q3 + theta)) / 40, sin(q3 + q4 + theta) / 40; (0), (9), ((3) * sin(theta)) / 40 - cos(q3 + q4 + theta) / 40 - (3 * cos(q1 + theta)) / 40 - (3 * cos(q3 + theta)) / 40 - cos(q1 + q2 + theta) / 40, - cos(q1 + q2 + theta) / 40 - (3 * cos(q1 + theta)) / 40, -cos(q1 + q2 + theta) / 40, - cos(q3 + q4 + theta) / 40 - (3 * cos(q3 + theta)) / 40, -cos(q3 + q4 + theta) / 40; sin(q1 + q2 + theta) / 40 + sin(q3 + q4 + theta) / 40 + (3 * sin(q1 + theta)) / 40 + (3 * sin(q3 + theta)) / 40 + ((3) * cos(theta)) / 40, ((3) * sin(theta)) / 40 - cos(q3 + q4 + theta) / 40 - (3 * cos(q1 + theta)) / 40 - (3 * cos(q3 + theta)) / 40 - cos(q1 + q2 + theta) / 40, cos(q2) / 100 + cos(q4) / 100 + (sqrt((109)) * cos(q3 + q4 - atan((3/10)))) / 800 - (3 * sqrt((109)) * cos(q1 + atan((3/10)))) / 800 + (3 * sqrt((109)) * cos(q3 - atan((3/10)))) / 800 - (sqrt((109)) * cos(q1 + q2 + atan((3/10)))) / 800 + (453/1600), cos(q2) / 100 - (3 * sqrt((109)) * cos(q1 + atan((3/10)))) / 1600 - (sqrt((109)) * cos(q1 + q2 + atan((3/10)))) / 1600 + (3/200), cos(q2) / 200 - (sqrt((109)) * cos(q1 + q2 + atan((3/10)))) / 1600 + (1/400), cos(q4) / 100 + (sqrt((109)) * cos(q3 + q4 - atan((3/10)))) / 1600 + (3 * sqrt((109)) * cos(q3 - atan((3/10)))) / 1600 + (3/200), cos(q4) / 200 + (sqrt((109)) * cos(q3 + q4 - atan((3/10)))) / 1600 + (1/400); sin(q1 + q2 + theta) / 40 + (3 * sin(q1 + theta)) / 40, - cos(q1 + q2 + theta) / 40 - (3 * cos(q1 + theta)) / 40, cos(q2) / 100 - (3 * sqrt((109)) * cos(q1 + atan((3/10)))) / 1600 - (sqrt((109)) * cos(q1 + q2 + atan((3/10)))) / 1600 + (3/200), cos(q2) / 100 + (3/200), cos(q2) / 200 + (1/400), (0), (0); sin(q1 + q2 + theta) / 40, -cos(q1 + q2 + theta) / 40, cos(q2) / 200 - (sqrt((109)) * cos(q1 + q2 + atan((3/10)))) / 1600 + (1/400), cos(q2) / 200 + (1/400), (1/400), (0), (0); sin(q3 + q4 + theta) / 40 + (3 * sin(q3 + theta)) / 40, - cos(q3 + q4 + theta) / 40 - (3 * cos(q3 + theta)) / 40, cos(q4) / 100 + (sqrt((109)) * cos(q3 + q4 - atan((3/10)))) / 1600 + (3 * sqrt((109)) * cos(q3 - atan((3/10)))) / 1600 + (3/200), (0), (0), cos(q4) / 100 + (3/200), cos(q4) / 200 + (1/400); sin(q3 + q4 + theta) / 40, -cos(q3 + q4 + theta) / 40, cos(q4) / 200 + (sqrt((109)) * cos(q3 + q4 - atan((3/10)))) / 1600 + (1/400), (0), (0), cos(q4) / 200 + (1/400), (1/400)];
end

function iDq_matrix = iDq(state)
    iDq_matrix = inv(Dq(state));
end

function Nqdq_matrix = Nqdq(state)
    % get the numerical value of states
    [theta, q1, q2, q3, q4] = deal(state(3), state(4), state(5), state(6), state(7));
    [theta_dot, q_dot_1, q_dot_2, q_dot_3, q_dot_4] = deal(state(10), state(11), state(12), state(13), state(14));
    % compute the numerical Nqda matrix of the robot
    Nqdq_matrix = [(q_dot_1^2 * cos(q1 + q2 + theta)) / 40 - (3 * theta_dot^2 * sin(theta)) / 40 + (q_dot_2^2 * cos(q1 + q2 + theta)) / 40 + (q_dot_3^2 * cos(q3 + q4 + theta)) / 40 + (q_dot_4^2 * cos(q3 + q4 + theta)) / 40 + (theta_dot^2 * cos(q1 + q2 + theta)) / 40 + (theta_dot^2 * cos(q3 + q4 + theta)) / 40 + (3 * q_dot_1^2 * cos(q1 + theta)) / 40 + (3 * q_dot_3^2 * cos(q3 + theta)) / 40 + (3 * theta_dot^2 * cos(q1 + theta)) / 40 + (3 * theta_dot^2 * cos(q3 + theta)) / 40 + (q_dot_1 * q_dot_2 * cos(q1 + q2 + theta)) / 20 + (q_dot_3 * q_dot_4 * cos(q3 + q4 + theta)) / 20 + (q_dot_1 * theta_dot * cos(q1 + q2 + theta)) / 20 + (q_dot_2 * theta_dot * cos(q1 + q2 + theta)) / 20 + (q_dot_3 * theta_dot * cos(q3 + q4 + theta)) / 20 + (q_dot_4 * theta_dot * cos(q3 + q4 + theta)) / 20 + (3 * q_dot_1 * theta_dot * cos(q1 + theta)) / 20 + (3 * q_dot_3 * theta_dot * cos(q3 + theta)) / 20; (3 * theta_dot^2 * cos(theta)) / 40 + (q_dot_1^2 * sin(q1 + q2 + theta)) / 40 + (q_dot_2^2 * sin(q1 + q2 + theta)) / 40 + (q_dot_3^2 * sin(q3 + q4 + theta)) / 40 + (q_dot_4^2 * sin(q3 + q4 + theta)) / 40 + (theta_dot^2 * sin(q1 + q2 + theta)) / 40 + (theta_dot^2 * sin(q3 + q4 + theta)) / 40 + (3 * q_dot_1^2 * sin(q1 + theta)) / 40 + (3 * q_dot_3^2 * sin(q3 + theta)) / 40 + (3 * theta_dot^2 * sin(q1 + theta)) / 40 + (3 * theta_dot^2 * sin(q3 + theta)) / 40 + (q_dot_1 * q_dot_2 * sin(q1 + q2 + theta)) / 20 + (q_dot_3 * q_dot_4 * sin(q3 + q4 + theta)) / 20 + (q_dot_1 * theta_dot * sin(q1 + q2 + theta)) / 20 + (q_dot_2 * theta_dot * sin(q1 + q2 + theta)) / 20 + (q_dot_3 * theta_dot * sin(q3 + q4 + theta)) / 20 + (q_dot_4 * theta_dot * sin(q3 + q4 + theta)) / 20 + (3 * q_dot_1 * theta_dot * sin(q1 + theta)) / 20 + (3 * q_dot_3 * theta_dot * sin(q3 + theta)) / 20 + (8829/100); q_dot_1 / 600 + q_dot_2 / 1200 + q_dot_3 / 600 + q_dot_4 / 1200 - (981 * cos(q1 + q2 + theta)) / 4000 - (981 * cos(q3 + q4 + theta)) / 4000 - (2943 * cos(q1 + theta)) / 4000 - (2943 * cos(q3 + theta)) / 4000 + ((2943) * sin(theta)) / 4000 + (9 * q_dot_1^2 * cos(q1)) / 1600 + (9 * q_dot_3^2 * cos(q3)) / 1600 + (3 * q_dot_1^2 * sin(q1)) / 160 - (q_dot_2^2 * sin(q2)) / 200 - (3 * q_dot_3^2 * sin(q3)) / 160 - (q_dot_4^2 * sin(q4)) / 200 + (3 * q_dot_1^2 * cos(q1 + q2)) / 1600 + (3 * q_dot_2^2 * cos(q1 + q2)) / 1600 + (3 * q_dot_3^2 * cos(q3 + q4)) / 1600 + (3 * q_dot_4^2 * cos(q3 + q4)) / 1600 + (q_dot_1^2 * sin(q1 + q2)) / 160 + (q_dot_2^2 * sin(q1 + q2)) / 160 - (q_dot_3^2 * sin(q3 + q4)) / 160 - (q_dot_4^2 * sin(q3 + q4)) / 160 + (9 * q_dot_1 * theta_dot * cos(q1)) / 800 + (9 * q_dot_3 * theta_dot * cos(q3)) / 800 - (q_dot_1 * q_dot_2 * sin(q2)) / 100 - (q_dot_3 * q_dot_4 * sin(q4)) / 100 + (3 * q_dot_1 * theta_dot * sin(q1)) / 80 - (q_dot_2 * theta_dot * sin(q2)) / 100 - (3 * q_dot_3 * theta_dot * sin(q3)) / 80 - (q_dot_4 * theta_dot * sin(q4)) / 100 + (3 * q_dot_1 * q_dot_2 * cos(q1 + q2)) / 800 + (3 * q_dot_3 * q_dot_4 * cos(q3 + q4)) / 800 + (3 * q_dot_1 * theta_dot * cos(q1 + q2)) / 800 + (3 * q_dot_2 * theta_dot * cos(q1 + q2)) / 800 + (3 * q_dot_3 * theta_dot * cos(q3 + q4)) / 800 + (3 * q_dot_4 * theta_dot * cos(q3 + q4)) / 800 + (q_dot_1 * q_dot_2 * sin(q1 + q2)) / 80 - (q_dot_3 * q_dot_4 * sin(q3 + q4)) / 80 + (q_dot_1 * theta_dot * sin(q1 + q2)) / 80 + (q_dot_2 * theta_dot * sin(q1 + q2)) / 80 - (q_dot_3 * theta_dot * sin(q3 + q4)) / 80 - (q_dot_4 * theta_dot * sin(q3 + q4)) / 80; - q1 / 600 - q2 / 1200 - theta_dot / 600 - (981 * cos(q1 + q2 + theta)) / 4000 - (2943 * cos(q1 + theta)) / 4000 - (9 * theta_dot^2 * cos(q1)) / 1600 - (q_dot_2^2 * sin(q2)) / 200 - (3 * theta_dot^2 * sin(q1)) / 160 - (3 * theta_dot^2 * cos(q1 + q2)) / 1600 - (theta_dot^2 * sin(q1 + q2)) / 160 - (q_dot_1 * q_dot_2 * sin(q2)) / 100 - (q_dot_2 * theta_dot * sin(q2)) / 100; (q_dot_1^2 * sin(q2)) / 200 - q2 / 1200 - theta_dot / 1200 - (981 * cos(q1 + q2 + theta)) / 4000 - q1 / 1200 + (theta_dot^2 * sin(q2)) / 200 - (3 * theta_dot^2 * cos(q1 + q2)) / 1600 - (theta_dot^2 * sin(q1 + q2)) / 160 + (q_dot_1 * theta_dot * sin(q2)) / 100; (3 * theta_dot^2 * sin(q3)) / 160 - q4 / 1200 - theta_dot / 600 - (981 * cos(q3 + q4 + theta)) / 4000 - (2943 * cos(q3 + theta)) / 4000 - (9 * theta_dot^2 * cos(q3)) / 1600 - (q_dot_4^2 * sin(q4)) / 200 - q3 / 600 - (3 * theta_dot^2 * cos(q3 + q4)) / 1600 + (theta_dot^2 * sin(q3 + q4)) / 160 - (q_dot_3 * q_dot_4 * sin(q4)) / 100 - (q_dot_4 * theta_dot * sin(q4)) / 100; (q_dot_3^2 * sin(q4)) / 200 - q4 / 1200 - theta_dot / 1200 - (981 * cos(q3 + q4 + theta)) / 4000 - q3 / 1200 + (theta_dot^2 * sin(q4)) / 200 - (3 * theta_dot^2 * cos(q3 + q4)) / 1600 + (theta_dot^2 * sin(q3 + q4)) / 160 + (q_dot_3 * theta_dot * sin(q4)) / 100];
end
