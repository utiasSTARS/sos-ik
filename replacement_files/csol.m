%%***************************************************************%%
%% csol: solve an sdp created by gendata2
%% sdp: sdp created by gendata2
%% soltag: specify solver [sdpt3|sqlp|sedumi] (optional)
%%
%% sedumi is not included in this package und needs to be installed
%%
%% Author: Tillmann Weisser
%%***************************************************************%%


function [sol, infocell] = csol(sdp,soltag)

if nargin < 2
    if isfield(sdp,'recy')
        if isfield(sdp.recy,'converted')
            soltag = sdp.recy.converted;
            fprintf('\n No solver specified. Use %s.\n',soltag)
        else
            fprintf('\n No solver specified. Use sdpt3.\n')
            soltag = 'sdpt3';
        end
    else
        fprintf('\n No solver specified. Use sqlp.\n')
        soltag = 'sqlp';
    end
else
    fprintf('\n Use %s.\n',soltag)
end


if strcmp(soltag,'sqlp')
    tstart=clock;
    [obj,X,y,Z,info,runhist] = sqlp(sdp.blk,sdp.At,sdp.C,sdp.b);
    tSOL = etime(clock,tstart);
elseif strcmp(soltag,'sdpt3')
    tstart=clock;
    [obj,X,y,Z,info,runhist, infocell] = sdpt3(sdp.blk,sdp.At,sdp.C,sdp.b);
    tSOL = etime(clock,tstart);
elseif strcmp(soltag,'sedumi')
    if isfield(sdp.recy,'converted')&&strcmp(sdp.recy.converted,'sedumi')
        tstart=clock;
        [x,y,info] = sedumi(sdp.At,sdp.b,sdp.C,sdp.blk,sdp.pars);
        tSOL = etime(clock,tstart);    
    else
        error('Requested solver is ''SeDumi'' but provided SDP is for SDPT3')
    end
else
    error('use one of the following solvertags: ''sqlp'', ''sdpt3'', ''sedumi'' ')
end

if strcmp(soltag,'sedumi')
    sol.obj = [sdp.C'*x, sdp.b'*y];
    sol.x = x;
    sol.y = y;
    sol.info = info;
else
    sol.obj = obj;
    sol.X = X;
    sol.y = y;
    sol.Z = Z;
    sol.info = info;
    sol.runhist = runhist;
end
    sol.soltag = soltag;
    sol.tSOL = tSOL;
