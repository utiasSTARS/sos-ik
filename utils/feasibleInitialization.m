function [P,s,lambda] = feasibleInitialization(arm,limits,dim)
%% Random initialization parameters
    
    dof = arm.dof;
    
    if dim==2
        
        lambda = limits'.*(2*rand(dof,1)-1);
        s =sqrt (abs( (2*(1-cos(limits))' - 2*(1-cos(lambda)))./( 2*(1-cos(limits))' ) ));
        P = arm.FKLinkPositions(lambda);
        
        
    elseif dim==3
        
        theta = limits'.*(2*rand(dof,1)-1) + 0.001;   
        phi = (pi/4)*(2*rand(dof,1) - ones(dof,1)); 
        lambda = [phi, theta];
        P = arm.FKLinkPositions(theta, (pi/4)*(2*rand(dof,1) - ones(dof,1)));   
        s =sqrt (abs( (2*(1-cos(limits))' - 2*(1-cos(theta)))./( 2*(1-cos(limits))' ) ));
        P = P(:,1:dof-2);
        
    end
    
end
