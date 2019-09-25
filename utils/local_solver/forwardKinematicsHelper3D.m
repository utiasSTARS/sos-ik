function [x_ee, x_penultimate] = forwardKinematicsHelper3D(q, link_lengths)
% Might need one less
d = length(link_lengths);

phi = q(1:d);
theta = q(d+1:end);

T = eye(4);
for idx = 1:d
%     T = T*DH(link_lengths(idx), 0, theta(idx), phi(idx));
    T = sphericalJointTransform(link_lengths(idx), theta(idx), phi(idx), T);
    if idx == (d-1)
        x_penultimate = [T(1,4); T(2,4); T(3,4)];
    end
end
x_ee = [T(1,4); T(2,4); T(3,4)];
end

