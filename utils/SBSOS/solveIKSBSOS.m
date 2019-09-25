function [data] = solveIKSBSOS(prob,params, n_max, gap_max, rank_tol, ccontrol_tol)
    
    dim = params.dim;
    dof = params.dof;
    arm = params.arm;
    limits = arm.limits; 
    goal = params.goal;
    solved = false;
    n = 1; 
    
    while ~solved && n <= n_max 
        
        [sol, psol, pop, info] = solver_sbsos(params);
        
        data.sol = sol;
        data.psol = postprocRAL(pop, psol.YY, rank_tol, ccontrol_tol);
        data.psol.obj = psol.obj;
        data.pop = pop;
        data.info = info;
        data.goal = params.goal;
        data.n = n;
        %data.G = params.G; 
        
        if data.psol.rnk == 1
            solved = true;
        else
            [P,s] = feasibleInitialization(arm,limits,dim);
            F = prob.armObjectives(P(:,1:dof-2),s);
            params.F = F; 
        end
        
        n=n+1;

    end
   
    data.feas = data.sol.info.gap < gap_max; 
    
end