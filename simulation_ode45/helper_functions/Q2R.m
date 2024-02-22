function R = Q2R(Q)
    R = (eye(3) + 2 * skew(Q(2:4)) * skew(Q(2:4)) - 2 * Q(1) * skew(Q(2:4)))';
end