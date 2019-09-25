function [E] = getNearestPointPartition(dim, dof, orientation_constraint)
%getNearestPointPartition Get the seed partition (i.e. not necessarily
%satisfying the RIP yet) for a dim-dimensional, dof-degree-of-freedom arm.


% Variable order:
% s0 x0 y0 s1 x1 y1 s2 s_orientation

if nargin < 3
    orientation_constraint = false;
end

% For now, don't include any subsets of larger blocks
E = {[1 expandIndex(1, dim)]};
N = dof-2;
for idx=1:N
    
    E_idx = [idx*(dim+1)+1 expandIndex(idx, dim)];
    if idx ~= 1
        E_idx = [E_idx expandIndex(idx-1, dim)];
    end
    if idx ~= N
        E_idx = [E_idx expandIndex(idx+1, dim)];
    end
    E{end+1} = sort(E_idx);
end

if orientation_constraint 
    M = dim+1;
    N = dof-3;
    E{end+1} = sort([expandIndex(dof-2,dim), 1 + M*(dof-2) + 1]); 
end

end


function [I] = expandIndex(idx, dim)
    I =  (1:dim) + idx*(dim+1) - dim;
end