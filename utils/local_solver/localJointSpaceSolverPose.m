function [sol] = localJointSpaceSolverPose(goal,goal_end, arm, q0, options)
%localJointSpaceSolver 

    limits = arm.limits;
    link_lengths = arm.l;
    lb = -limits;
    ub = limits;

    if nargin < 5
        options = optimoptions('fmincon');
        options = optimoptions(options,'Display','off');
        if exist('contains', 'builtin') == 5
            options = optimoptions(options, 'OptimalityTolerance', 1e-5);
            options.MaxFunctionEvaluations = 20000;
            options.MaxIterations = 10000;
        else
            options = optimoptions(options, 'TolFun', 1e-5);
            options.MaxFunEvals = 20000;
            options.MaxIter = 10000;
        end 
    end

    fun = @(x) norm(forwardKinematicsHelper(x, link_lengths(1:end-1)) - goal(1:2))^2;

    f_con = @(x) orientationConstraint(x, limits(end), goal_end(3));
    %f_con = @(x) orientationConstraintt(x, limits(end), goal, goal_end(1:2),link_lengths);


    [q_out, fval, exitflag] = fmincon(fun, q0(1:end), [], [], [], [], lb(1:end-1), ub(1:end-1), f_con, options);

    sol.q_out = q_out;
    sol.fval = fval;
    sol.exitflag = exitflag;  

end 


function [c, c_eq] = orientationConstraint(q, ang_lim, phi)

    c = abs(diff_angle(sum(q), phi)) - ang_lim;
    c_eq = [];

end

% function [c, c_eq] = orientationConstraintt(q, ang_lim, goal, goal_end, l)
%     x1 = forwardKinematicsHelper(q, l(1:end-1));
%     x2 = goal;
%     x3 = goal_end;
%     c = norm( (1/l(end))*(x3-x2) + (1/l(end-1))*(x2-x1) )^2 - 2*(1-cos(ang_lim));
%     %c = abs(diff_angle(sum(q), phi)) - ang_lim;
%     c_eq = [];

% end

