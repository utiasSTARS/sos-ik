classdef spatialManipulator
%SPATIALMANIPULATOR Summary of this class goes here
%   Detailed explanation goes here
    
    properties
        d
        a
        alpha
        theta
        dim
        dof
        limits
        l
    end
    
    methods
        function obj = spatialManipulator(a, d, alpha, theta, limits)
        %PLANARMANIPULATOR Construct an instance of this class
        %   Generates a manipulator in 3d space given DH parameters
            
            obj.dim = 3;
            obj.dof = length(a); 
            obj.limits = limits;
            
            obj.d = d;
            obj.a = a;
            obj.alpha = alpha;
            obj.theta = theta;
            obj.l = d;
            
        end
        
        function p = forwardKinematics(obj, theta, alpha, link_n)
        %FORWARDKINEMATICS Calculate FK of given link
        %   Returns position of end-effector given joint angles
            T = eye(4);
            
            for i = 1:link_n
                T = T*DH(obj.d(i), obj.a(i), alpha(i), theta(i));
            end
            
            p = [T(1,4); T(2,4) ; T(3,4)];
            
        end

        function [P] = FKLinkPositions(obj,phi,theta)

            d = obj.d;
            P = [];
            T = eye(4);
            for idx = 1:length(phi)
                T = sphericalJointTransform(d(idx), theta(idx), phi(idx), T);
                P = [P, T(1:3,4)];
            end
                
        end 
        
        function [phi, theta] = IKLinkPositions(obj,P)
        %IKLINKPOSITIONS Given link positions, find joint angles
            
            l = obj.l;
            P = [[0,0,0]', P];
            R = eye(3);
            
            phi = [];
            theta = [];
            
            for idx = 1:size(P,2)-1 
                p1 = P(:,idx);
                p2 = P(:,idx+1);
                
                d = norm(p2-p1);%d(idx);
                
                A = d.*R;
                B = p2-p1; 
                X = linsolve(A,B);
                
                theta_idx = real(acos(X(3)));
                theta = [theta, theta_idx ]; 
                
                phi_idx = atan2(X(2),X(1));
                phi = [phi, phi_idx];
                
                % update transforms
                Rz = [cos(phi_idx) -sin(phi_idx) 0;
                      sin(phi_idx) cos(phi_idx) 0;
                      0 0 1];
                
                Ry = [cos(theta_idx) 0 sin(theta_idx);
                      0 1 0;
                      -sin(theta_idx) 0 cos(theta_idx)]; 
                
                R = R*Rz*Ry; 
                
            end 
                
        end 
        
        function [D, Cm, Cn, A] = distanceMatrix(obj)
        % DISTANCEMATRIX returns an incomplete distance matrix describing
        % manipulator structure.
        %   D = [Cm, A; A', Cn];
            
            N = length(obj.alpha);
            
            Cm = zeros(N + 1, N + 1); % matrix describing the main graph
            Cn = zeros(N + 1, N + 1); % matrix describing the joint axis graph
            A = zeros(N + 1, N + 1); % connections between the two graphs
            
            
            T = eye(4);
            Z = [eye(3), [0;0;1]; zeros(1,3), 1];
            
            for idx = 1:N
                
                % main and axis transforms
                Tp = T; 
                Tpz = Tp * Z;
                T = Tp * DH(obj.d(idx), obj.a(idx), obj.alpha(idx), obj.theta(idx)); 
                Tz = T * Z;

                Cm(idx, idx + 1) = norm(T(1:3,4) - Tp(1:3,4));

                Cn(idx, idx + 1) = norm(Tz(1:3,4) - Tpz(1:3,4)); 
                
                A(idx, idx) = 1;
                A(idx, idx + 1) = norm(Tz(1:3,4) - Tp(1:3,4));
                A(idx + 1, idx) = norm(T(1:3,4) - Tpz(1:3,4)); 
                
            end

            Cm = Cm + Cm';
            Cn = Cn + Cn';
            A(end,end) = 1;

            D = [Cm, A; A', Cn];
        end
    

    end
end