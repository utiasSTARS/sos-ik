function G = jointLimitConstraint(alpha,l1,l2,ind,N,dim,s)
% JOINTLIMITCONSTRAINT - Generates a joint limit constraint 
% The joint limit constraint ||(1/l2)*(x2 - x1) - (1/l1)*(x1 - x0)||^2 <= 2*(1 - cos(alpha))
    
    if nargin <7
        s = false;
    end
    
    L = 2*(1-cos(alpha));

    M = dim + s*1; % displacement per point (+1 for aux variable)
    
    G1 = zeros(6*dim + 1 + s*1, N); % constraint matrix
    G1(end,end) = L;
    
    for idx = 1:dim
        G1((idx-1)*6 + 1, [s*1 + M*(ind(1)-1) + idx, end]) = [2, -l1^-2]; 
        G1((idx-1)*6 + 2, [s*1 + M*(ind(1)-1) + idx, s*1 + M*(ind(2)-1) + idx, end]) = [1,1,2*(l1*l2)^-1 + 2*l1^-2];
        G1((idx-1)*6 + 3, [s*1 + M*(ind(1)-1) + idx, s*1 + M*(ind(3)-1) + idx, end]) = [1,1,-2*(l1*l2)^-1];
        
        G1((idx-1)*6 + 4, [s*1 + M*(ind(2)-1) + idx, end]) = [2,-l2^-2 - 2*(l1*l2)^-1 - l1^-2];
        G1((idx-1)*6 + 5, [s*1 + M*(ind(2)-1) + idx, s*1 + M*(ind(3)-1) + idx, end]) = [1,1,2*(l1*l2)^-1 + 2*l2^-2];
        G1((idx-1)*6 + 6, [s*1 + M*(ind(3)-1) + idx, end]) = [2,-l2^-2];        
    end
    
    G1(:,end) = G1(:,end)./L;

    if s
        G1(end-1, [s*1 + M*(ind(2)-1) + M, end]) = [2,-1];
        G2 = G1;
        G2(:,end) = -G1(:,end); 
        G = {G1, G2};
    else
        G = G1;
    end

end
