clear all; close all; clc;
rand('seed', 23512436634723475376);
%% Randomized problem parameters
n_problems = 1000; % number of problems
closest_point = true; % use closest-point formulation

%% Manipulator
% Generate a planar (2d) manipulator
dof = 6; % number of dof 
dim = 2; % dimensionality

link_lengths = [2 2 1 2 3 2 4 4 1 2 2 4 4 1 1];
link_lengths = link_lengths(1:dof);

limits = pi*[1/4 1/4 1/8 1/4 1/4 1/2 1/4 1/4 1/2 1/8 1/8 1/8 1/8 1/8 1/8];
limits = limits(1:dof);

arm_len = sum(link_lengths); % maximum length
arm = planarManipulator(link_lengths, limits); 

%% Generate feasible goals in the environment 
% Generates feasible goals by sampling feasible configurations

goals = [];
oth = [];
for idx = 1:n_problems
    lambda = limits'.*(2*rand(dof,1)-1);
    P = arm.FKLinkPositions(lambda);
    phi = atan2(P(2,end) - P(2,end-1), P(1,end) - P(1,end-1));
    goals = [goals, [P(:,end); phi]];
    oth = [oth, P(:,end-1)];
end


%% Instantiate problem
% Generate constraints and objecives for SBSOS

prob_params.s = closest_point;
prob_params.b = [0;0];
prob = Problem(arm,prob_params);

% kinematic and obstacle constraints
G_arm = prob.linkageConstraints(); 

% joint limit constraints
G_lim = prob.jointLimitConstraints();

% add base location constraint
G_base = prob.quadraticAnchorEqualityConstraints([0,0]',link_lengths(1),1);

% add goal pose constraints
G_goal = {};
for idx = 1 : size(goals,2)    
    G_goal{end+1} = prob.poseConstraint(goals(1:dim,idx),goals(end,idx));  
end

%% Generate SBSOS parameters

% Sparsity pattern 
I = getNearestPointPartition(dim, dof,true);
% SBSOS parametes
[P,s,q0] = feasibleInitialization(arm,limits,dim);
F = prob.armObjectives(P(:,1:dof-2),s);

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



%% SBSOS
tic;
parfor idx = 1:size(goals,2) 
    

    [data{idx}] = solveIKSBSOS(prob, params{idx}, n_max, gap_max, ranktol, ccontroltol);
    
    if data{idx}.psol.rnk==1
        rank_1_list = [rank_1_list idx];
    end


end
T = toc; 

%% local solver
ext = [];
n_max_local = 1;
tic;
parfor idx=1:size(goals,2)
    fval = 100000;
    att = 0;
    q =  q0;
    while fval > 1e-4 && att < n_max_local
        data{idx}.local_sol = localJointSpaceSolverPose(oth(:,idx), goals(:,idx), arm, q(1:end-1)); 
        fval = data{idx}.local_sol.fval;
        att = att + 1;
        [P,s,q] = feasibleInitialization(arm,limits,dim); 
    end
    % disp(idx)
    ext = [ext data{idx}.local_sol.exitflag];
end
T_local = toc;

%% stats
analyze_sols;

