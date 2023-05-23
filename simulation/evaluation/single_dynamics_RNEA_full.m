theta_P1 = 0;
theta_P2 = 0;
w_P1 = 1;
w_P2 = 1;
dw_P1 = 2;
dw_P2 = 2;
eta_x = 0.1;
eta_y = 0.6;
d_eta_x = 0.1;
d_eta_y = 0.6;
dd_eta_x = 0.1;
dd_eta_y = 0.6;
l_pg = 1;
l_pa = 0.1;
m_a = 1;
m_p = 1;
B_V_B = [1; 2; 3];
B_dV_B = [2; 1; 1];

B_R_A = Ry(eta_y) * Rx(eta_x);
A_R_B = B_R_A';
P1_R_A = Rz(theta_P1)';
P2_R_A = Rz(theta_P2)';
J_P = diag([1 2 3]);
J_A = diag([3 2 1]);
J_B = diag([1 2 2]);
B_W_B = [1 2 3]';
B_dW_B = [1 1 3]';
A_W_AB = [d_eta_x d_eta_y 0]';
A_dW_AB = [dd_eta_x dd_eta_y 0]';
P_W_P1A = [0 0 w_P1]';
P_dW_P1A = [0 0 dw_P1]';
P_W_P2A = [0 0 -w_P2]';
P_dW_P2A = [0 0 -dw_P2]';
A_T_d = [1 4 3]';
A_T_f = [4 2 3]';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Original RNEA: Separated
% Forward dynamics
A_W_A = A_R_B * B_W_B + A_W_AB;
A_dW_A = A_R_B * B_dW_B + cross(A_W_A, A_W_AB) + A_dW_AB;
A_V_A = skew(Rx(eta_x)'*[0;0;-l_pg]) * A_R_B * B_W_B + A_R_B * B_V_B;
A_dV_A = skew(Rx(eta_x)'*[0;0;-l_pg]) * A_R_B * B_dW_B + A_R_B * B_dV_B + skew(A_V_A) * A_W_AB;

P_W_P1 = P1_R_A * A_W_A + P_W_P1A;
P_dW_P1 = P1_R_A * A_dW_A + cross(P_W_P1, P_W_P1A) + P_dW_P1A;
P_V_P1 = skew([0;0;-l_pa]) * P1_R_A * A_W_A + P1_R_A * A_V_A;
P_dV_P1 = skew([0;0;-l_pa]) * P1_R_A * A_dW_A + P1_R_A * A_dV_A + skew(P_V_P1) * P_W_P1A;

P_W_P2 = P2_R_A * A_W_A + P_W_P2A;
P_dW_P2 = P2_R_A * A_dW_A + cross(P_W_P2, P_W_P2A) + P_dW_P2A;
P_V_P2 = skew([0;0;l_pa]) * P2_R_A * A_W_A + P2_R_A * A_V_A;
P_dV_P2 = skew([0;0;l_pa]) * P2_R_A * A_dW_A + P2_R_A * A_dV_A + skew(P_V_P2) * P_W_P2A;

% Backward dynamics
P_T_P1 = J_P * P_dW_P1 + cross(P_W_P1, J_P * P_W_P1) + cross(P_V_P1, m_p * P_V_P1); 
P_T_P2 = J_P * P_dW_P2 + cross(P_W_P2, J_P * P_W_P2) + cross(P_V_P2, m_p * P_V_P2);
P_f_P1 = m_p * P_dV_P1 + skew(P_W_P1)*(m_p*P_V_P1);
P_f_P2 = m_p * P_dV_P2 + skew(P_W_P2)*(m_p*P_V_P2);

A_T_A = (P1_R_A' * P_T_P1 + P2_R_A' * P_T_P2) ...
   + J_A * A_dW_A + cross(A_W_A, J_A * A_W_A) + cross(A_V_A, m_a * A_V_A);
A_F_A = (P1_R_A' * P_f_P1 + P2_R_A' * P_f_P2) + m_a * A_dV_A + skew(A_W_A)*(m_a*A_V_A);
B_T_A = A_R_B' * A_T_A;
A_F_A1 = [A_T_A; A_F_A]

% (skew([0;0;-l_pg])*P1_R_A)' * P_T_P1 + (skew([0;0;-l_pg])*P_T_P2)' * P_F_P2 ...

%% Preparing
A_T_C = [Rx(eta_x)'*Ry(eta_y)' Rx(eta_x)'*[0;0;-l_pg];
         0 0 0 1];
dA_A =  [d_eta_x; d_eta_y; 0; 0; 0; 0];
ddA_A = [dd_eta_x; dd_eta_y; 0; 0; 0; 0];

P1_T_A = [Rz(theta_P1)' [0;0;-l_pa];
         0 0 0 1];
P2_T_A = [Rz(theta_P2)' [0;0;l_pa];
         0 0 0 1];
dA_P1 = [0; 0; w_P1; 0; 0; 0];
ddA_P1 = [0; 0; dw_P1; 0; 0; 0];
dA_P2 = [0; 0; -w_P2; 0; 0; 0];
ddA_P2 = [0; 0; -dw_P2; 0; 0; 0];


G_A = [J_A zeros([3 3]);
       zeros([3 3]) m_a * eye(3)];
G_P = [J_P zeros([3 3]);
       zeros([3 3]) m_p * eye(3)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Original RNEA
%% full dynamics
V_C  = [B_W_B; B_V_B];
dV_C = [B_dW_B; B_dV_B];

% Forward Iteration
V_A = Ad(A_T_C) * V_C + dA_A;
dV_A = Ad(A_T_C) * dV_C + ad(V_A) * dA_A + ddA_A;

V_P1 = Ad(P1_T_A) * V_A + dA_P1;
dV_P1 = Ad(P1_T_A) * dV_A + ad(V_P1) * dA_P1 + ddA_P1;

V_P2 = Ad(P2_T_A) * V_A + dA_P2;
dV_P2 = Ad(P2_T_A) * dV_A + ad(V_P2) * dA_P2 + ddA_P2;

% Backward Iteration
P_F_P1 = G_P * dV_P1 - ad(V_P1)' * (G_P * V_P1);
P_F_P2 = G_P * dV_P2 - ad(V_P2)' * (G_P * V_P2);
A_F_A2 = Ad(P1_T_A)' * P_F_P1 + Ad(P2_T_A)' * P_F_P2 + G_A * dV_A - ad(V_A)' * (G_A * V_A);
A_F_A2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simplified RNEA

r_pg = [0;0;l_pg];
A_R_P = P1_R_A';
B_R_P = B_R_A * A_R_P;
B_J_A = B_R_A * J_A * B_R_A';
B_J_P = B_R_P * J_P * B_R_P';
% Forward dynamcis
A_W_A = B_R_A' * B_W_B + A_W_AB;
A_dW_A = B_R_A' * B_dW_B + cross(A_W_A, A_W_AB) + A_dW_AB;
A_V_A = skew(Rx(eta_x)'*[0;0;-r_pg(3)]) * B_R_A' * B_W_B + B_R_A' * B_V_B;
A_dV_A = skew(Rx(eta_x)'*[0;0;-r_pg(3)]) * B_R_A' * B_dW_B + B_R_A' * B_dV_B + skew(A_V_A) * A_W_AB;

P_W_P1 = A_R_P' * A_W_A + P_W_P1A;
P_dW_P1 = A_R_P' * A_dW_A + cross(P_W_P1, P_W_P1A) + P_dW_P1A;
P_V_P1 = skew([0;0;-l_pa]) * A_R_P' * A_W_A + A_R_P' * A_V_A;
P_dV_P1 = skew([0;0;-l_pa]) * A_R_P' * A_dW_A + A_R_P' * A_dV_A + skew(P_V_P1) * P_W_P1A;

P_W_P2 = A_R_P' * A_W_A + P_W_P2A;
P_dW_P2 = A_R_P' * A_dW_A + cross(P_W_P2, P_W_P2A) + P_dW_P2A;
P_V_P2 = skew([0;0;l_pa]) * A_R_P' * A_W_A + A_R_P' * A_V_A;
P_dV_P2 = skew([0;0;l_pa]) * A_R_P' * A_dW_A + A_R_P' * A_dV_A + skew(P_V_P2) * P_W_P2A;


% Adverse Reactionary Moment
kA1 = (2*B_J_P + B_J_A) * B_dW_B ...
    + (2*B_J_P + B_J_A) * B_R_A * A_dW_AB ...
    + B_J_P * B_R_P * P_dW_P1A ...
    + B_J_P * B_R_P * P_dW_P2A;

% Gyroscopic Moment
B1 = B_W_B + B_R_A * A_W_AB;
BP1 = B1 + B_R_P * P_W_P1A;
BP2 = B1 + B_R_P * P_W_P2A;
kB1 = (2*B_J_P + B_J_A) * cross(B_W_B, B_R_A * A_W_AB) ...
    + B_J_P * cross(B_W_B + B_R_A * A_W_AB, B_R_P * P_W_P1A) ...
    + B_J_P * cross(B_W_B + B_R_A * A_W_AB, B_R_P * P_W_P2A) ...
    + cross(BP1, B_J_P * BP1) ... 
    + cross(BP2, B_J_P * BP2);

% Delta
kC1 = cross(B1, B_J_A * B1) ...
    + A_R_P * cross(P_V_P1, m_p * P_V_P1) ...
    + A_R_P * cross(P_V_P2, m_p * P_V_P2) ...
    + cross(A_V_A, m_a * A_V_A);

% Total Internal Moment from {A} to {B}
B_T_A = kA1 + kB1 + kC1;

% Translational
P_f_P1 = m_p * P_dV_P1 + skew(P_W_P1)*(m_p*P_V_P1);
P_f_P2 = m_p * P_dV_P2 + skew(P_W_P2)*(m_p*P_V_P2);
A_f_A = (A_R_P * P_f_P1 + A_R_P * P_f_P2) + m_a * A_dV_A + skew(A_W_A)*(m_a*A_V_A);
A_F_A3 = [A_R_B * B_T_A; A_f_A]

function AdT = Ad(T)
   AdT = [T(1:3, 1:3) zeros([3 3]);
          skew(T(1:3,4))*T(1:3, 1:3) T(1:3, 1:3)];
end

function adV = ad(V)
   adV = [skew(V(1:3)) zeros([3 3]);
          skew(V(4:6)) skew(V(1:3))];
end 
