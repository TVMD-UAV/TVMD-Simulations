A_R_B = Rx(0.1) * Ry(0.6) * Rz(0.3);
P_R_A = Rx(0.4) * Ry(0.2) * Rz(0.3);
J_P = diag([1 2 3]);
J_A = diag([3 2 1]);
J_B = diag([1 2 2]);
B_W_B = [1 2 3]';
B_dW_B = [1 1 3]';
A_W_AB = [2 2 3]';
A_dW_AB = [1 1 3]';
P_W_P1A = [0 1 3]';
P_dW_P1A = [1 2 1]';
P_W_P2A = [1 1 4]';
P_dW_P2A = [1 1 1]';
A_T_d = [1 4 3]';
A_T_f = [4 2 3]';

%% Original RNEA
% Forward dynamics
A_W_A = A_R_B * B_W_B + A_W_AB;
A_dW_A = A_R_B * B_dW_B + cross(A_W_A, A_W_AB) + A_dW_AB;

P_W_P1 = P_R_A * A_W_A + P_W_P1A;
P_dW_P1 = P_R_A * A_dW_A + cross(P_W_P1, P_W_P1A) + P_dW_P1A;

P_W_P2 = P_R_A * A_W_A + P_W_P2A;
P_dW_P2 = P_R_A * A_dW_A + cross(P_W_P2, P_W_P2A) + P_dW_P2A;

% Backward dynamics
P_T_P1 = J_P * P_dW_P1 + cross(P_W_P1, J_P * P_W_P1);
P_T_P2 = J_P * P_dW_P2 + cross(P_W_P2, J_P * P_W_P2);
A_T_A = P_R_A' * (P_T_P1 + P_T_P2) + J_A * A_dW_A + cross(A_W_A, J_A * A_W_A);
% B_T_B = A_R_B' * A_T_A + J_B * B_dW_B + cross(B_W_B, J_B * B_W_B);
B_T_A = A_R_B' * A_T_A;
% M_B = (J_B + B_J_A + B_J_P) \ (B_T_A - cross(B_W_B, (J_B + B_J_A + B_J_P) * B_W_B));

B_T = A_R_B' * (A_T_d + A_T_f);
M_B = J_B \ (B_T - B_T_A - cross(B_W_B, J_B * B_W_B));

%% Preparing 
P_R_B = P_R_A * A_R_B;
B_R_A = A_R_B';
B_R_P = P_R_B';

B_J_A = A_R_B' * J_A * A_R_B;
B_J_P = P_R_B' * J_P * P_R_B;

%% Simplified RNEA
B1 = B_W_B + B_R_A * A_W_AB;
BP1 = B1 + B_R_P * P_W_P1A;
BP2 = B1 + B_R_P * P_W_P2A;
kC1 = cross(B1, B_J_A * B1) ...
       + cross(BP1, B_J_P * BP1) ... 
       + cross(BP2, B_J_P * BP2);

kA1 = (2*B_J_P + B_J_A) * B_dW_B ...
   + (2*B_J_P + B_J_A) * B_R_A * A_dW_AB ...
   + B_J_P * B_R_P * P_dW_P1A ...
   + B_J_P * B_R_P * P_dW_P2A;

kB1 = (2*B_J_P + B_J_A) * cross(B_W_B, B_R_A * A_W_AB) ...
   + B_J_P * cross(B_W_B + B_R_A * A_W_AB, B_R_P * P_W_P1A) ...
   + B_J_P * cross(B_W_B + B_R_A * A_W_AB, B_R_P * P_W_P2A);

B_T_A2 = kA1 + kB1 + kC1;
M_B2 = J_B \ (B_T - B_T_A2 - cross(B_W_B, J_B * B_W_B));

B_T_A
B_T_A2
norm(B_T_A - B_T_A2)
norm(M_B - M_B2)
