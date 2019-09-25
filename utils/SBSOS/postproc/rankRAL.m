function [RANK, mineig] = rankRAL(YY, I, degfg, relTol)

%fprintf('\nranktol is %1.1e\n',ranktol)
p = length(YY);
EIGS = cell(1,p);
RANK = zeros(1,p);
for i = 1:p
    MM = momentmatrix(YY{i},length(I{i}),degfg);
    eigMM = eig(MM);  
    %validate
    smalltol = relTol*max(abs(eigMM));
    rankMM = length(find(abs(eigMM) > smalltol));
%     rankMM = length(find(abs(eigMM) > 1e-6));
    EIGS{i} = eigMM;
    RANK(i) = rankMM;
end
mineig = min(cellfun(@min,EIGS));
