%%
options = optimoptions('fmincon');
options = optimoptions(options,'Display','off');
options = optimoptions(options, 'OptimalityTolerance', 1e-3);
options.MaxFunctionEvaluations = 2000;
options.MaxIterations = 1000;

postproc = true;    

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
local_error_pos = [];
local_error_lim = [];
local_error_ang = [];

%refined
p_error_pos = [];
p_error_lim = [];
p_error_ang = [];

for idx = 1:length(data)
    local_fvals = [local_fvals data{idx}.local_sol.fval];
    feas = [feas, data{idx}.feas];
    goal = data{idx}.goal; 
    pnt = goal(1:dim) - arm.l(end)*[cos(goal(dim+1)); sin(goal(dim+1))];

    xsol = mom2x(data{idx}.psol.YY, data{idx}.pop); % grab SOS solution
    data{idx}.psol.X_solution = reshape(xsol(pointIndices(dim,dof)), dim, []); 
    
    X_sol = [data{idx}.psol.X_solution, pnt, goal(1:dim)]; % extracted sols + fixed points
    theta_sol = arm.IKLinkPositions(X_sol); % calculate joint angles 
        
    data{idx}.theta_sol = theta_sol; 
    X_theta_sol = arm.FKLinkPositions(theta_sol'); % re-calculate link positions
    
    if norm(X_theta_sol(:,end) - goal(1:dim)) <= 1e-2 && sum(abs(theta_sol) > limits) == 0 
        n = [n, data{idx}.n];
        error_pos = [error_pos, norm(X_theta_sol(:,end) - goal(1:dim))];
        error_ang = [error_ang, atan2(X_theta_sol(2,end) - X_theta_sol(2,end-1), X_theta_sol(1,end) - X_theta_sol(1,end-1)) - goal(dim+1)]; 
        error_lim = [error_lim, sum(abs(theta_sol) > limits)];        
        gaps = [gaps, data{idx}.sol.info.gap];

    elseif postproc 
            
        lc  = localJointSpaceSolverPose(pnt, goal, arm, theta_sol(1:end-1)', options);  
        theta_sol = lc.q_out;
        theta_sol = [lc.q_out; -diff_angle(sum(lc.q_out), goal(dim+1))]';
        X_theta_sol = arm.FKLinkPositions(theta_sol'); % re-calculate link positions
            
        if norm(X_theta_sol(:,end) - goal(1:dim)) <= 1e-2  && sum(abs(theta_sol) > limits) == 0
                
            n = [n, 6];
            p_error_pos = [p_error_pos, norm(X_theta_sol(:,end) - goal(1:dim))];
            error_pos = [error_pos, norm(X_theta_sol(:,end) - goal(1:dim))];
            p_error_ang = [p_error_ang, atan2(X_theta_sol(2,end) - X_theta_sol(2,end-1), X_theta_sol(1,end) - X_theta_sol(1,end-1)) - goal(dim+1)]; 
            p_error_lim = [p_error_lim, sum(abs(theta_sol) > limits)];        
            gaps = [gaps, data{idx}.sol.info.gap];
        end
            
    end
    
    local_sol = data{idx}.local_sol.q_out;
    local_sol = [local_sol; -diff_angle(sum(local_sol), goal(dim+1))];
    X_l = arm.FKLinkPositions(local_sol);
    
    if norm(X_l(:,end) - goal(1:dim)) <=1e-2
  
        
        local_error_ang = [local_error_ang, atan2(X_l(2,end) - X_l(2,end-1), X_l(1,end) - X_l(1,end-1)) - goal(dim+1) ]; 
        local_error_pos = [local_error_pos, sqrt(data{idx}.local_sol.fval)]; 
        local_error_lim = [local_error_lim, sum(abs(local_sol') > limits)];
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
