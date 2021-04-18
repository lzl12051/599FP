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

