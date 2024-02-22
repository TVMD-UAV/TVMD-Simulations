function [theta, u] = Q2theta_vector(Q)
    theta = 2 * acos(Q(1));
    u = Q(2:4) / sin(theta / 2);
end