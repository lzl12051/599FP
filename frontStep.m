% The robot will swing its front foot
function frontStep
    global yout tout
    
    H = [Qlist, Rlist];
    params.H = diag(H)

    Q02 = [1000, 1000, 1000, 10, 10, 10, 10];
    R2 = [1e-6, 1e-6];
    R2list = [];
    Q2list = [];

    for i = 1:10
        Q2list = [Q2list, Q02];
        R2list = [R2list, R2];
    end

    H2 = [Q2list, R2list];
    params.H2 = diag(H2)


end
