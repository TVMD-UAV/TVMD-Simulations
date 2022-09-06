function D = quaternion_multiplication(Q, P)
    D = [0; 0; 0; 0];
    D(1) = P(1)*Q(1) - Q(2:4)' * P(2:4);
    D(2:4) = Q(1) * P(2:4) + P(1) * Q(2:4) + skew(Q(2:4))*P(2:4);
end