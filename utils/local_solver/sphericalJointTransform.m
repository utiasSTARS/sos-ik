function T = sphericalJointTransform(d, theta, phi, T_in)
%sphericalJointTransform 

% Theta is elevation, phi is azimuth
if nargin < 3
    T_in = eye(4);
end

% d_in = T_in(1:3,1:3)*[0; 0; d];
% T_in(1:3, 4) = T_in(1:3, 4) + d_in;
% T_trans = eye(4);
% T_trans(3,4) = d;
% T = rotY(theta)*rotZ(phi)*T_in; %T_trans;

% T(1:3, 4) = T(1:3, 4) + T(1:3, 1:3)*[0; 0; d];

T = eye(4,4);
%T(1:3, 1:3) = rotY(theta)*rotZ(phi)*T_in(1:3,1:3);
T(1:3, 1:3) = T_in(1:3,1:3)*rotZ(phi)*rotY(theta);
T(1:3, 4) = T_in(1:3,4) + T(1:3,1:3)*[0; 0; d];

end
