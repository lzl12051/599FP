function funcs = impactMapping
    funcs.frontMapping = @frontMapping;
    funcs.doubleMapping = @doubleMapping;
    funcs.rearMapping = @rearMapping;
end

function frontMapping
    global yout
    x0 = yout(end, :);

    eq = dynaEq;
    dq_f = x0(8:14).';
    Dq_ = eq.Dq(x0); % double(subs(Dq));
    J_foot_back_ = eq.jacRfoot(x0); %double(subs(J_foot_back));
    J_foot_front_ = eq.jacFfoot(x0); % double(subs(J_foot_front));
    J_all = [J_foot_back_; J_foot_front_];
    %         7x1     7x7        7x4           4x7      7x7       7x4         4x7    7x1
    d_after = dq_f - inv(Dq_) * J_all.' * inv(J_all * inv(Dq_) * J_all.') * J_all * dq_f;
    yout(end, 8:14) = d_after;
end

function rearMapping
    global yout
    state_last_sec = yout(end, :).';
    eq = dynaEq;
    dq_f = state_last_sec(8:14);
    Dq_ = eq.Dq(state_last_sec); % (double(subs(Dq));
    J_foot_back_ = eq.jacRfoot(state_last_sec); % double(subs(J_foot_back));
    d_after = (dq_f - eq.iDq(state_last_sec) * J_foot_back_.' * inv(J_foot_back_ * inv(Dq_) * J_foot_back_.') * J_foot_back_ * dq_f);
    yout(end, 8:14) = d_after;
end

function doubleMapping
    global yout
    % XXX Not implemented
    global Dq J_foot_front J_foot_back
    eq = dynaEq;
    x_after = yout(end,:);


    dq_f = x_after(8:14).';
    Dq_ = eq.Dq(x_after);
    J_foot_back_ = eq.jacRfoot(x_after);
    J_foot_front_ = eq.jacFfoot(x_after);
    J_all = [J_foot_back_; J_foot_front_];

    d_after = dq_f - inv(Dq_) * J_all.' * inv(J_all * inv(Dq_) * J_all.') * J_all * dq_f;
    yout(end, 8:14) = d_after;

end
