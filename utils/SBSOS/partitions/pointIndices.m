function ind = pointIndices(dim,dof)
    
    ind = [];
    for idx = 1:dof-2
        ind = [ind, expandIndex(idx,dim)];
    end
    
    
end

function [I] = expandIndex(idx, dim)
    I =  (1:dim) + idx*(dim+1) - dim;
end