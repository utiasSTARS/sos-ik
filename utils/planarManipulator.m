classdef planarManipulator
%PLANARMANIPULATOR Kinematic model of a planar manipulator
    
    properties
        d
        a
        alpha
        dim
        dof
        limits
        l
    end
    
    methods
        function obj = planarManipulator(link_lengths, limits)
        %PLANARMANIPULATOR Construct an instance of this class
            
            obj.dim = 2;
            obj.dof = length(link_lengths);
            obj.limits = limits;

            obj.d = 0.*zeros(obj.dof,1);
            obj.a = link_lengths;
            obj.alpha = 0.*ones(obj.dof,1); 
            obj.l = link_lengths;
        end
        
        function p = forwardKinematics(obj, q, l)
        %FORWARDKINEMATICS Calculate FK of given link
            
            l = obj.l(1:length(q));
            q_cum = cumsum(q).';
            x_local = [cos(q_cum).*l; sin(q_cum).*l];
            p = sum(x_local, 2); 
        end
        
        function P = FKLinkPositions(obj,theta)
        %FKLINKPOSITIONS Given joint angles, find link positons
            l = obj.l;
            P = [];
            for idx = 1:length(theta)
                P = [P, obj.forwardKinematics(theta(1:idx), l(1:idx))];
            end
        end
        
        function theta = IKLinkPositions(obj, P)
        %IKLINKPOSITIONS Given link positions, find joint angles
            l = obj.l;
            P = [[0,0]', P];
            
            R = eye(2);
            theta = [];
            
            for idx = 1 : length(l)
                p1 = P(:,idx);
                p2 = P(:,idx+1);
                d = norm(p2-p1);
                
                A = d.*R;
                B = p2 - p1;
                
                X = linsolve(A,B);
                
                theta_idx = atan2(X(2), X(1)); 
                
                theta = [theta, theta_idx];
                
                Rz = [cos(theta_idx) -sin(theta_idx);
                      sin(theta_idx) cos(theta_idx)];
                
                R = R*Rz;
            end 
        end 
        
        function [D, Cm, Cn, A] = distanceMatrix(obj)
        % DISTANCEMATRIX returns an incomplete distance matrix describing
        % manipulator structure.
        % D = [Cm, A; A', Cn];
            
            D = diag(obj.l', 1) + diag(obj.l', 1)';

            Cm = D;
            Cn = [];
            A = [];
        end 
    
    end
end

