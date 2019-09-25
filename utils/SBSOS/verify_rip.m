function [rip_holds] = verify_rip(I)
%verify_rip 

rip_holds = true;
for idx=1:length(I)-1
    rip_holds = false;
    I_next = I{idx+1};
    I_union = [];
    for jdx=1:idx
        I_union = [I_union I{jdx}];
    end
    I_intersection = intersect(I_next, I_union);
    for kdx=1:idx
        if all(ismember(I_intersection, I{kdx}))
            rip_holds = true;
            break;
        end
    end
    % Break if it failed for 1 instance
    if ~rip_holds
        return
    end
end

end

