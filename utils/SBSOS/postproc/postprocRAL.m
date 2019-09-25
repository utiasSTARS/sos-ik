
%%
%% Author: Tillmann Weisser (modified by Matt Giamou Sept. 
%%***************************************************************%%

function psolRAL = postprocRAL(pop, YY, rank_tol, ccontrol_tol)
k = pop.k;
[RANK,mineig] = rankRAL(YY,pop.I,k,rank_tol);
psolRAL.rnk = sum(RANK)/length(RANK);
psolRAL.ranks = RANK;
psolRAL.mineig = mineig;
psolRAL.YY = YY;
if (psolRAL.rnk ==1)%&&(mineig > -10^6)
    psolRAL.xsol = mom2x(YY,pop);
    ccontrol=zeros(length(pop.G),1);
    for j = 1:length(pop.G)
        ccontrol(j) = evalpoly(pop.G{j},psolRAL.xsol);
    end
    psolRAL.fx=evalpoly(pop.F,psolRAL.xsol);
    psolRAL.ccontrol = ccontrol;
    
    if max(abs(ccontrol) > ccontrol_tol)
        psolRAL.valid = false;
    else
        psolRAL.valid = true;
    end
else 
    psolRAL.valid = false;
end

end
