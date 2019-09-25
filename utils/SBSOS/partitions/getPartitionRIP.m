function [I] = getPartitionRIP(E, clean, parent_seed)
%getPartitionRIP 
%

if nargin < 2
    clean = true;
end

if nargin < 3
    parent_seed = false;
end
N = max(max(E));
M = size(E, 1);

%% Use edges as seeds (covering partition)
if ~parent_seed
    I = cell(1, M);
    I_min = zeros(1, M);
    for m=1:M % go through all edges

        idx = E(m, 1); % indice of first
        jdx = E(m, 2); % indice of second

        I{m} = sort([idx jdx]); % store indices for m-th edge (sort in ascending order)
        I_min(m) = min(I{m}); % store the smallest indice for this edge
    end
else
%% Use parents of each vertex (edges e(1) -> e(2) for parent definition)
    I = cell(1, N);
    I_min = zeros(1, N);
    for n=1:N
        I{n} = sort(E(E(:,2) == n, 1).');
        % Don't need to consider 'roots' (?)
        if ~isempty(I{n})
            I_min(n) = min([n I{n}]);
        else
            I_min(n) = -1;
        end
    end
    I(I_min == -1) = [];
    I_min(I_min == -1) = [];
end

%% Sort so that increases by minimal element
[alpha, ind_sort] = sort(I_min); % sort the smallest indices in ascending order
I = I(ind_sort); % sort the actual edges in ascending order of their lowest indice

%% Form Junction Tree to satisfy running intersection property (RIP)
I = junction_tree(I, alpha, clean); % alpha is the actual array of sorted minimal elements.

end

