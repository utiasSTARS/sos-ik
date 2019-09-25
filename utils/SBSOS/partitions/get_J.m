function [J] = get_J(I,G)
%get_J From a partition I and sbsos constraints G form the constraint
%partition J.

m = length(G);
n = length(I);
J = cell(n, 1);
for idx=1:n
    J_idx = [];
    for jdx=1:m
        constraint = G{jdx};
        present_variables = find(any(constraint(:, 1:end-1)));
%         if ~isempty(intersect(I{idx}, present_variables))
        intersection_ij = intersect(I{idx}, present_variables);
        if length(intersection_ij) == length(present_variables)
            if all(sort(intersection_ij) == sort(present_variables))
                J_idx = [J_idx jdx];
            end
        end
    end
    J{idx} = J_idx;
end

