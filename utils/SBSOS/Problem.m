% PROBLEM - Creates a graph (matrix of edge objects) of the problem
% 
%   arm - arm object containing DH parameters
%   obs - array of obstacle objects  
%   n_sbs - density of subsampling for enforcing constraints
%
%
%   Author: Filip Maric, May 2019, Zagreb
%---------------------------------------------------------------------------

classdef Problem < handle

    properties
        X
        Cm
        Cn
        A
        I = {};
        arm
        dim
        dof
        n_vr
        limits
        params
    end
    
    methods
        
        function obj = Problem(m, params) 
            obj.arm = m; %6.53
            obj.limits = m.limits;
            obj.dof = m.dof;
            obj.dim = m.dim;
            obj.params = params;
            
            [obj.X, obj.Cm, obj.Cn, obj.A] = m.distanceMatrix(); 
            
            if m.dim == 2
                obj.n_vr = m.dim*(m.dof-2) + 1 + params.s*(m.dof); % added one more s
            elseif m.dim == 3
                obj.n_vr = m.dim*(m.dof-2) + 1 + params.s*(m.dof); 
            end

        end 
        
        function G = linkageConstraints(obj)
        % LINKAGECONSTRAINTS - linkage constraints.
        % Polynomial constraints enforcing vertices to be at a fixed distance.
        %----------------------------------------------------------------------------- 
            n_vr = obj.n_vr;
            dim = obj.dim;
            dof = obj.dof;
            a = obj.arm.l;
            s = obj.params.s; 
            X = obj.X;
            
            G = {};
            
            for idx = 1 : dof-3 
                l = a(idx + 1); 
                G(end+1:end+2) = distanceEqConstraint(l, [idx,idx+1], n_vr, dim, s); 
            end
            
        end 
        
        function G = jointLimitConstraints(obj)
        % JOINTLIMITCONSTRAINTS - symmetrical joint limit constraints
        % Polynimals constrainting joint angles by constraining distances.
        %----------------------------------------------------------------------------- 
            n_vr = obj.n_vr;
            dim = obj.dim;
            dof = obj.dof;
            a = obj.arm.l;
            b = obj.params.b;
            limits = obj.limits;
            M = dim + 1;
            
            G = {};
            
            % base joint constraint  (confirmed correct)
            alpha = limits(1);
            l = a(1);
            if dim==2
                u = [1,0]'; 
            else
                u = [0,0,1]';
            end
            L = 2*(1-cos(alpha));
            
            G1 = zeros(2*dim + 1 + 1, n_vr);  
            for idx = 1:dim
                G1(2*(idx-1) + 1, [1 + idx, end]) = [2, -1/(l^2)];
                G1(2*(idx-1) + 2, [1 + idx, end]) = [1, (2/l^2)*b(idx) + u(idx)*(2/l)];
            end

            G1(end, end) = -(1/l^2)*b'*b - (2/l)*u'*b - 1 + L; 
            G1(:,end) = G1(:,end)./L;
            
            G1(end-1, [1, end]) = [2,-1]; 
            G{end+1} = G1;
            G1(:,end) = -G1(:,end);
            G{end+1} = G1; 
            
            % joint at x0 (confirmed correct)
            alpha = limits(2);
            l0 = a(1);
            l1 = a(2);
            L = 2*(1-cos(alpha));
                
            G1 = zeros(5*dim + 1 + 1, n_vr);
            for idx = 1:dim
                G1(5*(idx-1) + 1, [1 + idx, end]) = [2,  -1/l0^2 - 2/(l1*l0) - 1/l1^2];
                G1(5*(idx-1) + 2, [1 + idx, end]) = [1,  b(idx)*(2/l0^2 + 2/(l0*l1))];
                G1(5*(idx-1) + 3, [1 + idx, 1 + M + idx, end]) = [1, 1, 2/l1^2 + 2/(l1*l0)];
                G1(5*(idx-1) + 4, [1 + M + idx, end]) = [1, -b(idx)*2/(l1*l0)];
                G1(5*(idx-1) + 5, [1 + M + idx, end]) = [2, -1/l1^2];
            end
                
            G1(end,end) = -(1/l0^2)*b'*b + L;
            G1(:,end) = G1(:,end)./L;

            G1(end-1,[1 + M, end]) = [2,-1];
            G{end+1} = G1;
            G1(:,end) = -G1(:,end);
            G{end+1} = G1; 
            
            % inner joints surrounded by other inner joints
            for idx = 2:dof-3
                alpha = limits(idx+1);

                l1 = a(idx);
                l2 = a(idx+1);
                
                Gl = jointLimitConstraint(alpha,l1,l2,[idx-1,idx,idx+1], n_vr, dim, true);
                
                G(end+1:end+2) = Gl;
        
            end 
        end
        
        function G = finalJointLimitConstraints(obj, p, r)
            
            n_vr = obj.n_vr;
            dim = obj.dim;
            dof = obj.dof;
            a = obj.arm.l;
            limits = obj.limits;
            
            M = dim + 1;
            
            G = {};
            
            alpha = limits(end-1);
            L = 2*(1-cos(alpha));
            l0 = a(end-2);
            l1 = a(end-1);
            g = p;
            
            G1 = zeros(5*dim + 1 + 1, n_vr);
            for idx = 1:dim
                G1(5*(idx-1) + 1, [1 + M*(dof-3) + idx, end]) = [2,  -1/l0^2 - 2/(l1*l0) - 1/l1^2];
                G1(5*(idx-1) + 2, [1 + M*(dof-3) + idx, end]) = [1,  g(idx)*(2/l1^2 + 2/(l0*l1))];
                G1(5*(idx-1) + 3, [1 + M*(dof-3) + idx, 1 + M*(dof-4) + idx, end]) = [1, 1, 2/l0^2 + 2/(l1*l0)];
                G1(5*(idx-1) + 4, [1 + M*(dof-4) + idx, end]) = [1, -g(idx)*2/(l1*l0)];
                G1(5*(idx-1) + 5, [1 + M*(dof-4) + idx, end]) = [2, -1/l0^2];
            end
            
            G1(end,end) = -(1/l1^2)*g'*g + L;   
            G1(:,end) = G1(:,end)./L;
            
            G1(end-1,[1 + M*(dof-2), end]) = [2,-1];
            
            G{end+1} = G1;
            G1(:,end) = -G1(:,end); 
            G{end+1} = G1; 
            
            if ~isnan(limits(end));
                % G2
                if dim==2
                    u = [cos(r), sin(r)]';
                else
                    phi = r(1);
                    theta = r(2);
                    u = [sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta)]';
                end
                
                alpha = limits(end);
                L = 2*(1-cos(alpha));
                
                G2 = zeros(2*dim + 1 + 1, n_vr); 
                for idx = 1:dim
                    G2(2*(idx-1) + 1, [1 + M*(dof-3) + idx, end]) = [2, -1/(l1^2)];
                    G2(2*(idx-1) + 2, [1 + M*(dof-3) + idx, end]) = [1, (2/l1^2)*g(idx) - u(idx)*(2/l1)];
                end

                G2(end, end) = -(1/l1^2)*g'*g + (2/l1)*u'*g - 1 + L; 
                G2(:,end) = G2(:,end)./L; 
            
                G2(2*dim+1, [1 + M*(dof-2) + 1, end]) = [2,-1]; 
                G{end+1} = G2;
                G2(:,end) = -G2(:,end);
                G{end+1} = G2; 
            end

        end
        
        function G = quadraticAnchorEqualityConstraints(obj, anc, l, inds)
        % ANGLEEQUALITYCONSTRAINTS
        %----------------------------------------------------------------------------- 
            dim = obj.dim;
            dof = obj.dof;
            n_vr = obj.n_vr;
            s = obj.params.s;
            G = {};
            
            M = dim + s*1;
            
            for idx=1:length(inds)
                ind = inds(idx);
                
                G1 = zeros(2*dim + 1, n_vr);
                for jdx = 1:dim
                    G1((jdx-1)*2 + 1, [s*1 +  M*(ind-1) + jdx, end]) = [2,1];
                    G1((jdx-1)*2 + 2, [s*1 +  M*(ind-1) + jdx, end]) = [1, -2*anc(jdx,idx)];
                end

                G1(end, end) = anc(:,idx)'*anc(:,idx) - l^2;
                G1(:,end) = G1(:,end)./(l^2 + (l==0)*(1 - l^2)); % scaling might be infeasible
                G{end+1,1} = G1;
                G1(:,end) = -G1(:,end);
                G{end+1,1} = G1;
                
            end
        end 
        
        function G = poseConstraint(obj, p, r)
            n_vr = obj.n_vr;
            dim = obj.dim;
            dof = obj.dof;
            a = obj.arm.l;
            limits = obj.limits;
            
            G = {};
            
            if dim == 2
                theta = r(1);
                p = p - a(end)*[cos(theta), sin(theta)]';
            elseif dim==3
                phi = r(1);
                theta = r(2);
                p = p - a(end)*[sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta)]';
            end
            G(end+1:end+2) = obj.quadraticAnchorEqualityConstraints(p , a(end-1), dof-2);
            G(end+1:end+2 + ~isnan(limits(end))*2) = obj.finalJointLimitConstraints(p, r); 
            
        end 
        
        function G = positionConstraint(obj, p)
            n_vr = obj.n_vr;
            dim = obj.dim;
            dof = obj.dof;
            a = obj.arm.l;
            limits = obj.limits;
            
            G = {};
            
            G(end+1:end+2) = obj.quadraticAnchorEqualityConstraints(p , a(end), dof-1);
            G(end+1:end+2) = obj.finalJointLimitConstraints(p, []);
            
        end 


        function F = armObjectives(obj, anc, theta)
            
            n_vr = obj.n_vr;
            dim = obj.dim;
            dof = obj.dof;
            limits = obj.limits;
            s = obj.params.s;
            
            M = dim + s*1;
            
            F = [];
            scl = 0;
            
            for ind = 1:size(anc,2)
                
                % Generate objective for vertices
                Fk = zeros(2*dim, n_vr);
                for idx = 1:dim
                    Fk(2*(idx-1) + 1,[s*1 + M*(ind-1) + idx, end]) = [2,1];
                    Fk(2*(idx-1) + 2,[s*1 + M*(ind-1) + idx, end]) = [1,-2*anc(idx,ind)];
                end
                
                scl = scl + anc(1:dim,ind)'*anc(1:dim,ind); 
                F = [F; Fk];
            end                    
            
            if s
                % Generate objective for aux variables
                Fk = zeros(2*dof, n_vr);
                sscl = 0;
                for idx = 1:dof-1
                    Fk(2*(idx-1) + 1, [1 + M*(idx-1), end]) = [2,1];
                    Fk(2*(idx-1) + 2, [1 + M*(idx-1), end]) = [1,-2*theta(idx)];
                    sscl = sscl + theta(idx)^2;
                end
                
                if ~isnan(limits(end))
                    Fk(2*(dof-2) + 3, [1 + M*(dof-2) + 1, end]) = [2,1];
                    Fk(2*(dof-2) + 4, [1 + M*(dof-2) + 1, end]) = [1,-2*theta(end)];
                    sscl = sscl + theta(end)^2;
                end

                scl = scl + sscl;
                
                F = [F; Fk];
            end
            

            F = [F; [zeros(1, n_vr - 1), scl]];            
        end 
        
    end

end