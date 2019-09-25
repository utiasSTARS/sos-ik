function [I_out] = makePartitionRIP(I, clean)
% makePartitionRIP 
%
% Inputs:
% I - cell array containing row vectors with indices representing
% partitions of problem variables
%
% clean - boolean flag indicating whether to clean the sequence in the
% Junction Tree algorithm (should almost always do this)
if verify_rip(I)
    I_out = I;
    return
end
    
if nargin < 2
    clean = true;
end

I_min = zeros(size(I));
for idx=1:length(I)
    I_min(idx) = min(I{idx});
end

%% Sort so that increases by minimal element
[alpha, ind_sort] = sort(I_min); % sort the smallest indices in ascending order
I = I(ind_sort); % sort the actual edges in ascending order of their lowest indice

%% Form Junction Tree to satisfy running intersection property (RIP)
I_out = junctionTree(I, alpha, clean); % alpha is the actual array of sorted minimal elements.


end

