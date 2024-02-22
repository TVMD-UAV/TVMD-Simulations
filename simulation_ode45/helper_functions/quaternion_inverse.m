function Q2 = quaternion_inverse(Q1)
    Q2 = Q1;
    Q2(2:4) = -Q2(2:4);
end