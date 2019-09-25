function [sol, psol, pop, info] = solver_sbsos(params)
%feasibility_sbsos

%% Unwrap params
    dof = params.dof;
    dim = params.dim;
    G = params.G;
    if isfield(params, 'F')
        F = params.F;
        n = (size(F,2)-1)/dim;
        %N = dim*n;
        N = params.N;
        s = params.s;
    else
        n = dof-1;
        N = dim*n;
        %         F = zeros(1, N+1);
        F = zeros(1, size(G{1}, 2));
    end

    if isfield(params, 'k')
        k = params.k;
    else
        k = 1;
    end

    if isfield(params, 'M_max')
        M_max = params.M_max;
    else
        M_max = n + 1;
    end

    if isfield(params, 'd')
        d = params.d;
    else
        d = 1;
    end

    if isfield(params, 'hierarchy')
        % Can be 'BSOS', 'SBSOS', or 'SPUT'
        hierarchy = params.hierarchy;
    else
        hierarchy = 'SBSOS';
    end

    if isfield(params, 'I')
        I_unexpanded = makePartitionRIP(params.I);
    elseif isfield(params, 'E')
        I_unexpanded = getGenericPartitionRIP(params.E);
    else
        % This default already satisfies the RIP
        I_unexpanded = cell(n-1, 1);
        for idx=1:n-1
            I_unexpanded{idx} = [idx idx+1];
        end
    end

    if isfield(params, 'solver') 
        solver = params.solver;
    else
        % Can be 'sdpt3', 'sedumi', or 'sqlp'
        solver = 'sdpt3';
    end

    I = I_unexpanded;
    %% Expand I with dimension
    % I = cell(length(I_unexpanded), 1);
    
    % for idx=1:length(I_unexpanded)
    %     I{idx} = sort(unique(I_unexpanded{idx}));
    % end
    % if s
    %     K = dim + 1;
    % else
    %     K= dim;
    % end
    
    % I = cell(length(I_unexpanded), 1);
    % for idx=1:length(I_unexpanded)
    %     Cidx = [];
    %     vals = unique(I_unexpanded{idx});
    %     for val=vals
    %         Cidx = [Cidx K*(val-1)+1:K*(val-1)+K];
    %     end
    %     Cidx = sort(unique(Cidx));
    %     I{idx} = Cidx;
    % end
    %% Verify RIP
    % Should remove after testing 
    %assert(verify_rip(I), 'RIP not satisfied!'); 
    
    %% Construct J
    J = get_J(I, G);
    
    %% Solve with SBSOS 
    pop.n = N;% dim*n;
    pop.F = F;
    pop.G = G;
    pop.I = I;
    pop.J = J;
    % Can k even be zero for linear (constant) cost function??? 
    pop.k = k;
    pop.d = d;
    sdp = gendata2(pop, hierarchy);
    [sol, info] = csol(sdp, solver); % Try 'sedumi' too
    psol = postproc(pop,sdp,sol);
end