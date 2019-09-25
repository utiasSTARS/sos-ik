function d_ang = diff_angle(a, b)

d_ang = a - b;
d_ang = mod((d_ang + pi), 2*pi) - pi;

end