function [x_ee] = forwardKinematicsHelper(q, link_lengths)
    q_cum = cumsum(q).';
    x_local = [cos(q_cum).*link_lengths; sin(q_cum).*link_lengths];
    x_ee = sum(x_local, 2);
end