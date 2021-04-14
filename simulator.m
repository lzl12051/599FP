function simulator()
    global tout yout;
   

    function continue_ODE = constraintsCheck(time_stamp, state, sys_input)
        % This function will check the state of the robot
        % If any constraint is violated, continueODE will be zero to stop the ODE function

        % Set constraints
        max_torque = 70; % unit Nm
        min_torque = -70;

        max_knee_angle = deg2rad(170); % need radian
        min_knee_angle = deg2rad(-170);
        % XXX should not be [state(5) state(7)], this is the joint velocity
        knee_angles = []; % q2 q4

        max_hip_angle = deg2rad(170); % need radian
        min_hip_angle = deg2rad(-30);
        hip_angles = []; % q1 q3

        max_joint_velovity = 20;
        joints_velocities = abs(state(4:7));

        if any(sys_input > max_torque) || (any(sys_input < min_torque))
            continue_ODE = 0;
            warning(['Torque(s) over limit: ',num2str(sys_input) ', limit: ',num2str(min_torque),',', num2str(max_torque)])
            return
        end
        
        if any(knee_angles > max_knee_angle) || (any(knee_angles < min_knee_angle))
            continue_ODE = 0;
            warning(['Knee angle(s) over limit: ',num2str(knee_angles) ', limit: ',num2str(min_knee_angle),',', num2str(max_knee_angle)])
            return
        end

        if any(hip_angles > max_hip_angle) || (any(hip_angles < min_hip_angle))
            continue_ODE = 0;
            warning(['Hip angle(s) over limit: ',num2str(hip_angles) ', limit: ',num2str(min_hip_angle),',', num2str(max_hip_angle)])
            return
        end

        if any(joints_velocities > max_joint_velovity)
            continue_ODE = 0;
            warning(['Joint(s) velocity over limit: ', num2str(joints_velocities.') ', limit: ',num2str(max_joint_velovity),',', num2str(max_joint_velovity)])
            return
        end

        continue_ODE = 1;
    end

end

function animate(t, x, ts, filename)
    % Simulate and generate a video of the given tout, yout
    %   t: the simulation time stamps related to yout data point
    %   x: yout data points
    %   ts: framerates, an integer
    %   filename: name of the generated video file

    % sample function
    function [Et, Ex] = evenSample(t, x, Fs)
        %       CONVERTS A RANDOMLY SAMPLED SIGNAL SET INTO AN EVENLY SAMPLED
        %       SIGNAL SET (by interpolation)
        % Obtain the process related parameters
        N = size(x, 2); % number of signals to be interpolated
        M = size(t, 1); % Number of samples provided
        t0 = t(1, 1); % Initial time
        tf = t(M, 1); % Final time
        EM = (tf - t0) * Fs; % Number of samples in the evenly sampled case with
        % the specified sampling frequency
        Et = linspace(t0, tf, EM)';

        % Using linear interpolation (used to be cubic spline interpolation)
        % and re-sample each signal to obtain the evenly sampled forms
        for s = 1:N
            Ex(:, s) = interp1(t(:, 1), x(:, s), Et(:, 1));
        end

    end

    % Drawing function
    function drawQuadruped2D(p, st)
        % Set up configuration values
        a = 0.15; b = 0.5; l = 0.2;
        x = st(1); y = st(2); theta = st(3);
        q1 = st(4); q2 = st(5);
        q3 = st(6); q4 = st(7);

        % Get the positions
        cmb = [x; y];
        mid_lower_body_link = cmb + [a / 2 * sin(theta); -a / 2 * cos(theta)];
        mid_upper_body_link = cmb + [-a / 2 * sin(theta); a / 2 * cos(theta)];
        top_left = mid_upper_body_link + [-b / 2 * cos(theta); -b / 2 * sin(theta)];
        top_right = mid_upper_body_link + [b / 2 * cos(theta); b / 2 * sin(theta)];

        joint3 = mid_lower_body_link + [-b / 2 * cos(theta); -b / 2 * sin(theta)];
        joint1 = mid_lower_body_link + [b / 2 * cos(theta); b / 2 * sin(theta)];
        cm3 = joint3 + [-l / 2 * cos(q3 + theta); -l / 2 * sin(q3 + theta)];
        joint4 = joint3 + [-l * cos(q3 + theta); -l * sin(q3 + theta)];
        rear_foot = joint4 + [-l * cos(theta + q3 + q4); -l * sin(theta + q3 + q4)];
        cm4 = joint4 + [-l / 2 * cos(theta + q3 + q4); -l / 2 * sin(theta + q3 + q4)];
        joint2 = joint1 + [-l * cos(theta + q1); -l * sin(theta + q1)];
        cm2 = joint1 + [-l / 2 * cos(theta + q1); -l / 2 * sin(theta + q1)];
        front_foot = joint2 + [-l * cos(theta + q1 + q2); -l * sin(theta + q1 + q2)];
        cm1 = joint2 + [-l / 2 * cos(theta + q1 + q2); -l / 2 * sin(theta + q1 + q2)];

        % Draw the robot
        plot([rear_foot(1), joint4(1)], [rear_foot(2), joint4(2)], 'b-', 'LineWidth', 2)
        hold on
        plot([mid_upper_body_link(1), mid_lower_body_link(1)], [mid_upper_body_link(2), mid_lower_body_link(2)], "b--")
        axis equal
        scatter(joint1(1), joint1(2))
        plot([top_left(1), top_right(1)], [top_left(2), top_right(2)], 'r-', 'LineWidth', 2)
        plot([top_left(1), joint3(1)], [top_left(2), joint3(2)], 'r-', 'LineWidth', 2)
        plot([joint1(1), top_right(1)], [joint1(2), top_right(2)], 'r-', 'LineWidth', 2)
        plot([joint3(1), joint1(1)], [joint3(2), joint1(2)], 'r-', 'LineWidth', 2)
        plot([joint1(1), joint2(1)], [joint1(2), joint2(2)], 'g-', 'LineWidth', 2)
        plot([joint3(1), joint4(1)], [joint3(2), joint4(2)], 'g-', 'LineWidth', 2)
        plot([front_foot(1), joint2(1)], [front_foot(2), joint2(2)], 'b-', 'LineWidth', 2)
        scatter(cm4(1), cm4(2), 'ro')
        scatter(cm3(1), cm3(2), 'ro')
        scatter(cm2(1), cm2(2), 'ro')
        scatter(cm1(1), cm1(2), 'ro')
        scatter(cmb(1), cmb(2), 'rx', 'LineWidth', 1.5)
        grid
        hold off;
    end

    [te, xe] = evenSample(t, x, 1 / ts);
    figure(1);
    axes1 = axes;
    %save as a video
    spwriter = VideoWriter(filename);
    set(spwriter, 'FrameRate', ts, 'Quality', 100);
    open(spwriter);

    fig1 = figure(1);

    figure_x_limits = [-1 1];
    figure_y_limits = [-1 1];

    axes1 = axes;
    set(axes1, 'XLim', figure_x_limits, 'YLim', figure_y_limits);
    set(axes1, 'Position', [0 0 1 1]);
    set(axes1, 'Color', 'w');

    for k = 1:length(te)
        % drawone(axes1, xe(k, :)');
        % drawFunction(axes1, xe(k, :)');
        drawQuadruped2D(axes1, xe(k, :)');
        set(axes1, 'XLim', figure_x_limits, 'YLim', figure_y_limits);
        drawnow;
        pause(ts);
        frame = getframe(gcf);
        writeVideo(spwriter, frame);
    end

end
