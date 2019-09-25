function [I_out] = junctionTree(I, alphas, clean)
%junctionTree Run the algorithm from Junction Trees Construction in
% Bayesian Networks (Smail, 2017)
if nargin < 3
    clean = true;
end
%% Form {Qi} from I
r = length(unique(alphas)); % len of array of lowest indices without repetition
Q = cell(1,r); % Q's as in paper, number of subsets equal to number of unique lowest indices
alph_ind = 1; % indice of current subset
last_min = alphas(1); % get into the first lowest indice
for idx = 1:length(I) % go through all edges
    if alphas(idx) ~= last_min % if current lowest indice is different than last
        alph_ind = alph_ind + 1; % continue going through indices
        last_min = alphas(idx); % set last smallest indice as current
    end
    if isempty(Q{alph_ind}) % indices of Q are sequential and not determined by the actual smallest indice generating Q
        Q{alph_ind} = I{idx}; % set subset Q as current indices       
    else
        Q{alph_ind} = [Q{alph_ind} I{idx}]; % add new indices SUSPECT I THINK YOU CAN GET REPEATING LOWEST INDICES IN Q THIS WAY
    end
end

% Remove redundant values CLEARS UP HERE I THINK
alphas = zeros(1, length(Q));
for idx=1:length(Q)
    Q{idx} = sort(unique(Q{idx})); % remove repeating lowest indices in Q and sort 
    alphas(idx) = Q{idx}(1); % fill array of lowest indice defining each Q
end

%% Clean the sequence Q to make it proper (no sets subsets of others)
%Use fact that they are sorted for fast comparison
if clean % are we making sure some Q's aren't actually completely contained in other Q's? damn right we are
    del_indices = [];
    for idx=2:length(Q) % for each Q starting from the second
        for jdx=1:idx-1 % see all previous Q's
            if ~ismember(jdx, del_indices) % is this Q already deleted?
                if is_subset(Q{idx}, Q{jdx}) % returns true if Qi is a subset of Qj
                    del_indices = [del_indices idx]; % add to deleted and bail
                    break;
                end
            end
        end
    end
    Q(del_indices) = []; % actually wipe deleted Q's, it think it does that
    % for ind=del_indices
    %     Q{ind} = [];
    % end
    alphas(del_indices) = []; % wipe lowest indices of delete Q's too
end
%% Run algorithm A from Smail, 2017
% So now we have a proper sequence of Q's, what do
r = length(Q); 
D{1} = Q{1};
S{1} = Q{1}(Q{1} ~= 1);
alphas = [alphas r+1];


% Iterate to r, or N? Or r+1? 
for jdx = 2:r %N
    Dj = Q{jdx}; % initialize Dj as Qj
    for kdx=1:jdx-1 % if k < j as per condition
        sk = min(S{kdx});
        if isempty(sk) % not sure what this is, safety?
            sk = inf;
        end
        if sk >= alphas(jdx) && sk < alphas(jdx+1) % check if lowest indice is between that of Q(j) and Q(j+1)
            Dj = [Dj S{kdx}]; % if yes, unite! BUT WATCH OUT FOR REPETITION
        end
    end
    Dj = sort(unique(Dj)); % TOOK CARE OF REPETION
    D{jdx} = Dj;
    if isempty(Dj)
        S{jdx} = [];
    else
        Sj = [];
        for kdx=jdx+1:r
            Sj = [Sj Q{kdx}]; % union of all  Qk, k>j
        end
        Sj = sort(intersect(Dj, unique(Sj))); % take unique elements , get intersection with  Dj
        S{jdx} = Sj;
    end
end
Cs = fliplr(D); % since D is flipped the wrong way ...

%% Part B - make the sequence proper
k = length(Cs);
p_exists = true;
C = Cs;

while p_exists
    % Check p exists
    [p, k] = get_p(C);
    p_exists = ~isempty(p);
    if p_exists
        % Replace elements of C
        remove_inds = [p];
        %Cp_save = C{p};
        %C{p} = C{k};
        C{k} = C{p};
        for idx=k+1:p-1
            if is_subset(C{idx}, Cp_save)
                remove_inds = [remove_inds idx];
            end
        end
        C(remove_inds) = [];
%         p_new = p; % - length(remove_inds);
%         %C{p} = C{k};
%         %C{p_new} = C{k- length(remove_inds)};
%         for ldx=p_new+1:length(C)
%             C{ldx} = C{ldx-p_new};
%         end
%         C(p_new+1:length(C)) = C(1:length(C)-p_new);
    end
end


I_out = C;

end

function [p, k] = get_p(C)
    n = length(C);
    for jdx=2:n
        for idx=1:jdx-1
            if is_subset(C{idx}, C{jdx})
                p = jdx;
                k = idx;
                return
            end
        end
    end
    
    p = [];
    k = [];
end

function [flag] = is_subset(A, B)
% Determine if A is a subset of B
inds = arrayfun(@(x) ~isempty(find(B==x, 1)), A);
flag = all(inds); 

end