function G = distanceEqConstraint(l,ind,N,dim,s)
%DISTANCEEQCONSTRAINT 
%   l - fixed distance between two vertices
%   ind - indices of the two vertices + the s variable if used
%   N - total number of variables in problem
%   dim - dimension problem is embedded in 
    
    if nargin < 5
        s = false;
    end
    
    M = dim + s*1;
    G1 = zeros(3*dim + 1, N);
    G1(end,end)= -1;
    
    for idx = 1:dim
        G1((idx-1)*3 + 1, [s*1 + M*(ind(1)-1) + idx, end]) = [2, (l^2)^-1]; 
        G1((idx-1)*3 + 2, [s*1 + M*(ind(1)-1) + idx, s*1 + M*(ind(2)-1) + idx, end]) = [1,1,-2*(l^2)^-1];
        G1((idx-1)*3 + 3, [s*1 + M*(ind(2)-1) + idx, end]) = [2,(l^2)^-1];
    end

    G2 = G1;
    G2(:,end) = -G1(:,end);
    
    G = [{G1}, {G2}];
    
end