function G = fullPolyMat(cfs, sclr, ind, n_vr, dim)
% FULLPOLYMAT - Full representative matrix of a local pairwise representation.
% Assumes index 1 is an anchor and doesn't include it in problem
%
%   cfs - local polynomial representation
%   sclr - scalar part
%   ind - indices of local variables in problem
%   n_vr - total number of variables in problem
%   dim - dimension the points are embedded in
%
%    
%   Author: Filip Maric, Apr. 2019, Toronto
%-----------------------------------------------------------------------------
    
%   empty local monomial matrix corresponding to coeff vector
    for n = 3:3:30
        if nchoosek(n+2-1,2)+n+1 - length(cfs) == 0
            G_empty = emptyMonomialMatrix(n/3);
            break;
        end
    end

    % generate local matrix from coeff vector 
    rws = find(cfs); % non-zero rows
    rw_sum = sum(G_empty(rws,:));
    cls = find(rw_sum); % non-zero cols 
    G_lc = [G_empty(rws, cls), cfs(rws)]; % remove local variables which don't appear at all
    
    % generate global matrix
    G = zeros(length(rws), n_vr);
    for ldx = dim-1:-1:0
        for kdx = ind 
            G(:, dim*kdx - ldx) = G_lc(:,1);
            G_lc = G_lc(:, 2:end);
        end
    end   
    G(:, end) = G_lc(:,1);
end
