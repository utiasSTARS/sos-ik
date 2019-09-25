function [sol] = localJointSpaceSolverPose3D(goal,goal_end,arm,phi0,theta0, options)
%localJointSpaceSolver 

% Inputs
% goal - position of the penultimate joint
% goal_end - position of the end effector (actual final joint)
% arm - arm containing link lengths in field a and joint limits 

if nargin < 6
    options = optimoptions('fmincon');
    options = optimoptions(options,'Display','off'); 
    if exist('contains', 'builtin') == 5
        options = optimoptions(options, 'OptimalityTolerance', 1e-5);
        options.MaxFunctionEvaluations = 3000;
        options.MaxIterations = 10000; 
    else
        options = optimoptions(options, 'TolFun', 1e-5);
        options.MaxFunEvals = 3000;
        options.MaxIter = 10000;
    end
end

% Phi is azimuth (about z), theta is elevation (about y)

% We have DOF limits and link lengths
limits = arm.limits;
link_lengths = arm.l; %arm.a';
goal = goal(1:3);

% Only DOF-1 variables are used (because final is fixed and angle
% constrained) 
% lb = [-inf*ones(size(limits(1:end-1))) -limits(1:end-1)];
% ub = [inf*ones(size(limits(1:end-1))) limits(1:end-1)];
lb = [-inf*ones(size(limits(1:end-1))) -limits(1:end-1)];
ub = [inf*ones(size(limits(1:end-1))) limits(1:end-1)];

fun = @(x) norm(forwardKinematicsHelper3D(x, link_lengths(1:end-1)) - goal)^2;

f_con = @(x) orientationConstraint(x, limits(end), goal_end, link_lengths);

q0 = [phi0; theta0];
[q_out, fval, exitflag] = fmincon(fun, q0, [], [], [], [], lb, ub, f_con, options);

sol.q_out = q_out;
sol.fval = fval;
sol.exitflag = exitflag;

end

function [c, c_eq] = orientationConstraint(q, ang_lim, goal_end, link_lengths)
    [x_ee, x_penult] = forwardKinematicsHelper3D(q, link_lengths(1:end-1));
    c = norm((goal_end-x_ee)/link_lengths(end) - (x_ee-x_penult)/link_lengths(end-1))^2 - 2*(1-cos(ang_lim));
    c_eq = [];
end