%clear all; close all; clc;
%rand('seed', 2764622801740);

%% Randomized problem parameters
%n_problems = 100; % number of problems will be n_samples^dim
closest_point = true; % use closest-point formulation

%% Manipulator
% Generate a spatial (3d) manipulator from link lenghts.

dim = 3; % dimensionality
         %dof = 10; % degrees of freedom

%link_lengths = [2 2 1 2 3];
%link_lengths = [2 2 1 2 3 2 4];
link_lengths = [2 2 1 2 3 2 4 4 1 2 2 4 4 1 1];
link_lengths = link_lengths(1:dof);

%limits =  pi*[1/4 1/4 1/8 1/4 1/4];
%limits =  pi*[1/4 1/4 1/8 1/4 1/4 1/2 1/4];
limits = pi*[1/4 1/4 1/8 1/4 1/4 1/2 1/4 1/4 1/2 1/8 1/8 1/8 1/8 1/8 1/8];
limits = limits(1:dof);

arm_len = sum(link_lengths); % maximum length
arm = spatialManipulator(zeros(dof,1), link_lengths, zeros(dof,1), zeros(dof,1), limits);

%% Generate feasible goals in the environtment 
% Generate n_samples^2 points in the environment spanning a 
% box in [-arm_len, arm_len]x[-arm_len, arm_len].

goals = [];
oth = [];
for idx = 1:n_problems
    lambda = limits'.*(2*rand(dof,1)-1) + 0.001;    
    P = arm.FKLinkPositions((pi/4)*(2*rand(dof,1) - ones(dof,1)), lambda);
    
    phi = atan2(P(2,end) - P(2,end-1), P(1,end) - P(1,end-1));
    theta = acos( (P(3,end) - P(3,end-1)) /  norm( P(:,end) - P(:,end-1) ));  
    
    goals = [goals, [P(:,end); phi; theta]];
    oth = [oth, P(:,end-1)];
end

%% Instantiate problem
% Instantiate problem object which generates constraints and
% objectives for SBSOS.

prob_params.s = closest_point;
prob_params.b = [0, 0, 0]';
prob = Problem(arm,prob_params);

% kinematic and obstacle constraints
G_arm = prob.linkageConstraints(); 

% joint limit constraints
G_lim = prob.jointLimitConstraints();

% add base location constraint
G_base = prob.quadraticAnchorEqualityConstraints([0,0,0]',link_lengths(1),1);

% add goal location constraint for every goal instance
G_goal = {};
for idx = 1 : size(goals,2) 
    G_goal{end+1} = prob.poseConstraint(goals(1:dim,idx),goals(dim+1:end,idx));
end

%% Generate SBSOS parameters

% Sparsity pattern 
I = getNearestPointPartition(dim, dof, true);
%I = {1:1:prob.n_vr-1};
% SBSOS parametes
[P,s, q0] = feasibleInitialization(arm,limits,dim);
F = prob.armObjectives(P,s);

for idx = 1 : size(goals,2) 
    
    G = {G_arm{:}, G_lim{:}, G_base{:}, G_goal{idx}{:}}'; 
    
    params{idx}.dof = dof;
    params{idx}.dim = dim;
    params{idx}.arm = arm;
    params{idx}.goal = goals(:,idx);
    params{idx}.k = 1;
    params{idx}.d = 1;
    params{idx}.I = I;
    params{idx}.G = G;
    params{idx}.F = F(:,:); 
    params{idx}.M_max = arm_len; 
    params{idx}.N = prob.n_vr - 1;
    params{idx}.s = prob_params.s;
    params{idx}.hierarchy = 'SBSOS'; 
end

%% Solve problems

gap_max = 0.1; % max allowed gap
n_max = 1; % max attempts
ranktol = 10e-3; % rank tolerance
ccontroltol = 10e-2; % constraint violation tolerance

data = cell(size(goals,2),1); 
rank_1_list = [];

% SBSOS
tic;
parfor idx = 1:size(goals,2) 
    
    [data{idx}] = solveIKSBSOS(prob, params{idx}, n_max, gap_max, ranktol, ccontroltol);
    
    if data{idx}.psol.rnk==1
        rank_1_list = [rank_1_list idx];
    end

end
T = toc;

%% Local solver
ext = [];
n_max_local = 20;
tic;
parfor idx=1:size(goals,2)
    fval = 100000;
    att = 0;
    q =  q0;
    % In case SBSOS is commented out 
    data{idx}.goal = goals(:,idx);
    while fval > 1e-4 && att < n_max_local
        data{idx}.local_sol = localJointSpaceSolverPose3D(oth(:,idx), goals(1:3,idx), arm, q(1:end-1,1), q(1:end-1,2)); 
        fval = data{idx}.local_sol.fval;
        att = att + 1;
        [~,~,q] = feasibleInitialization(arm,limits,dim);
        %         ext = [ ext, data{idx}.local_sol.exitflag];
    end
    data{idx}.local_sol.att = att;
end 
T_local = toc;

%% Local solver active-set
% ext = []; 
% options = optimoptions('fmincon');
% if exist('contains', 'builtin') == 5
%     options = optimoptions(options, 'OptimalityTolerance', 1e-3);
%     options.MaxFunctionEvaluations = 2000;
%     options.MaxIterations = 10000;
% else
%     options = optimoptions(options, 'TolFun', 1e-3);
%     options.MaxFunEvals = 2000;
%     options.MaxIter = 10000;
% end

% tic;
% parfor idx=1:size(goals,2)
%     fval = 100000;
%     att = 0;
%     idx
%     q =  q0;
%     while fval > 1e-2 && att < 5
%         data{idx}.local_sol_active_set = localJointSpaceSolverPose3D(oth(:,idx), goals(1:3,idx), arm, q(1:end-1,1), q(1:end-1,2), options); 
%         fval = data{idx}.local_sol_active_set.fval;
%         att = att + 1;
%         [~,~,q] = feasibleInitialization(arm,limits,dim);
% %         ext = [ ext, data{idx}.local_sol_active_set.exitflag];
%     end
% end 
% T_local_active_set = toc;
% %% Extract solutions and check 
% X_theta_sol = {}; 
% error_pos = [];
% error_ang = [];
% errors_lim = [];
% feas = [];
% gaps = [];
% n = [];

% for idx = 1:length(data)
    
%     if data{idx}.psol.valid
        
%         goal = data{idx}.goal;
%         phi = goal(dim+1);
%         theta = goal(dim+2);
%         pnt = goal(1:dim) - arm.a(end)*[sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta)]';
        
%         data{idx}.psol.X_solution = reshape(data{idx}.psol.xsol(pointIndices(dim,dof)), dim, []);
       
        
%         X_sol = [data{idx}.psol.X_solution, oth(:,idx), goal(1:dim)]; % extracted sols + fixed points
%         [phi_sol, theta_sol] = arm.IKLinkPositions(X_sol);
        
%         X_theta_sol{end+1} = arm.FKLinkPositions(phi_sol,theta_sol); % re-calculate link positions
        
%         error_pos = [error_pos, norm(X_theta_sol{end}(:,end) - goal(1:dim))];
%         error_ang = [error_ang, abs(sum(theta_sol) - goal(dim+2))]; 
%         errors_lim = [errors_lim, sum(abs(theta_sol) > limits)];
        
%         gaps = [gaps, data{idx}.sol.info.gap];
%         n = [n, data{idx}.n];
%     end
    
%     feas = [feas, data{idx}.feas];
    
% end