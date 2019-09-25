%%
options = optimoptions('fmincon');
% options = optimoptions(options,'Display','off');
if exist('contains', 'builtin') == 5
    options = optimoptions(options, 'OptimalityTolerance', 1e-5);
    options.MaxFunctionEvaluations = 3000;
    options.MaxIterations = 1000; 
else
    options = optimoptions(options, 'TolFun', 1e-5);
    options.MaxFunEvals = 3000;
    options.MaxIter = 1000;
end

postproc = false;
%% Extract solutions and check 
X_theta_sol = {}; 
error_pos = [];
error_ang = [];
error_lim = [];
feas = [];
gaps = [];
n = [];

% Local info
local_fvals = [];
local_fvals_computed = [];
local_error_pos = [];
local_error_lim = [];
local_error_ang = [];

local_error_sizes = [];

error_sizes = [];

parfor idx = 1:length(data)
    local_fvals = [local_fvals data{idx}.local_sol.fval];
    feas = [feas, data{idx}.feas];
    goal = data{idx}.goal; 
    phi = goal(dim+1);
    theta = goal(dim+2);
    pnt = goal(1:dim) - arm.l(end)*[sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta)]';

    xsol = mom2x(data{idx}.psol.YY, data{idx}.pop); % grab SOS solution 
    data{idx}.psol.X_solution = reshape(xsol(pointIndices(dim,dof)), dim, []); 

    X_sol = [data{idx}.psol.X_solution, pnt, goal(1:dim)]; % extracted sols + fixed points
    [phi_sol, theta_sol] = arm.IKLinkPositions(X_sol); % calculate joint angles 
        
    data{idx}.theta_sol = theta_sol;
    data{idx}.phi_sol = phi_sol; 
    X_theta_sol = arm.FKLinkPositions(phi_sol,theta_sol); % re-calculate link positions
        
    if norm(X_theta_sol(:,end) - goal(1:dim)) <= 1e-2
            
        n = [n, data{idx}.n];
        error_pos = [error_pos, norm(X_theta_sol(:,end) - goal(1:dim))];
        error_ang = [error_ang, atan2(X_theta_sol(2,end) - X_theta_sol(2,end-1), X_theta_sol(1,end) - X_theta_sol(1,end-1)) - goal(dim+1)]; 
        error_lim = [error_lim, sum(abs(theta_sol) > limits)];
        % Limits are broken if element is < 0 
        error_sizes = [error_sizes limits - abs(theta_sol)];
        gaps = [gaps, data{idx}.sol.info.gap];

    elseif postproc
        
        lc  = localJointSpaceSolverPose3D(pnt, goal(1:dim), arm, phi_sol(1:end-1)', theta_sol(1:end-1)', options);  
        phi_sol = lc.q_out(1:dof-1);
        theta_sol = lc.q_out(dof:end); 
        X = arm.FKLinkPositions(phi_sol,theta_sol);
        
        phi_sol = [phi_sol; atan2(goal(2) - X(2,end),goal(1) - X(1,end))];
        theta_sol = [theta_sol; acos( (goal(3)- X(3,end))/ norm(goal(1:dim) - X(:,end)))];
        
        X_theta_sol = arm.FKLinkPositions(phi_sol,theta_sol);
        
        if norm(X_theta_sol(:,end) - goal(1:dim)) <= 1e-2
            n = [n, 6];
            error_pos = [error_pos, norm(X_theta_sol(:,end) - goal(1:dim))];
            error_ang = [error_ang, atan2(X_theta_sol(2,end) - X_theta_sol(2,end-1), X_theta_sol(1,end) - X_theta_sol(1,end-1)) - goal(dim+1)]; 
            error_lim = [error_lim, sum(abs(theta_sol) > limits)];
            error_sizes = [error_sizes abs(abs(theta_sol) - limits)];
            gaps = [gaps, data{idx}.sol.info.gap];
        end
        
    end

    lc = data{idx}.local_sol.q_out;
    phi_sol = data{idx}.local_sol.q_out(1:dof-1);
    theta_sol = data{idx}.local_sol.q_out(dof:end); 
    X = [arm.FKLinkPositions(phi_sol, theta_sol), goal(1:3)]; 
    [phi_sol, theta_sol] = arm.IKLinkPositions(X);
    X_theta_sol = arm.FKLinkPositions(phi_sol,theta_sol);
 
    local_fvals_computed = [local_fvals_computed norm(X_theta_sol(:,end) - goal(1:dim))^2];
    if norm(X_theta_sol(:,end) - goal(1:dim)) <= 1e-2 && data{idx}.local_sol.att <= 1
        
        local_error_ang = [local_error_ang,abs( atan2(X_theta_sol(2,end) -X_theta_sol(2,end-1),X_theta_sol(1,end) -X_theta_sol(1,end-1)) - goal(dim+1)) ]; 
        local_error_pos = [local_error_pos, sqrt(data{idx}.local_sol.fval)]; 
        local_error_lim = [local_error_lim, sum(abs(theta_sol(dof:end)') > limits)];
        local_error_sizes = [local_error_sizes abs(abs(theta_sol(dof:end)' - limits))];
    end
    
    
    
end 

local_num_sol = length(local_error_pos);
num_sol = length(error_pos);

local_mean_error_pos = mean(local_error_pos);
mean_error_pos = mean(error_pos);

local_num_cvl = length(find(local_error_lim > 0));
num_cvl = length(find(error_lim > 0));


figure(1);
bar(error_pos);
hold on;
plot(xlim,[mean_error_pos mean_error_pos], 'r');
title("SOS-IK position error");
legend("Position error", "Mean position error");
xlabel("problem number")
ylabel('$ \|\mathbf{x}_{N} - \mathbf{g} \| $','Interpreter','Latex')
hold off;

figure(2);
bar(local_error_pos);
hold on;
plot(xlim,[local_mean_error_pos local_mean_error_pos], 'r');
title("fmincon position error");
legend("Position error", "Mean position error");
xlabel("problem number")
ylabel('$ \|\mathbf{x}_{N} - \mathbf{g} \| $','Interpreter','Latex')
hold off;
