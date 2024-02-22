Q1 = theta_vector2Q(pi/3, [1;0;0]);
Q2 = theta_vector2Q(pi/2, [1;0;0]);

R = Q2R(Q1)
R * [0;1;0]
Q1 = quaternion_inverse(Q1);


tilde_Q = quaternion_multiplication(Q1, Q2);
[theta, u] = Q2theta_vector(tilde_Q)

Q2R(tilde_Q)
Q2R2(tilde_Q)