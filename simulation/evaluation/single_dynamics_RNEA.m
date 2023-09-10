% syms A_R_B  [3 3] matrix
% syms P_R_A  [3 3] matrix

% syms B_W_B  [3 1] matrix
% syms B_dW_B [3 1] matrix

% syms A_W_AB [3 1] matrix
% syms A_dW_AB [3 1] matrix

% syms P_W_PA [3 1] matrix
% syms P_dW_PA [3 1] matrix

% syms J_P  [3 3] matrix
% syms J_A  [3 3] matrix
% syms J_B  [3 3] matrix

% syms P_T_d  [3 1] matrix

% A_R_B = eye([3 3]);
% P_R_A = eye([3 3]);
% J_P = eye([3 3]);
% J_A = eye([3 3]);
% J_B = eye([3 3]);
% B_W_B = ones([3 1]);
% B_dW_B = ones([3 1]);
% A_W_AB = ones([3 1]);
% A_dW_AB = ones([3 1]);
% P_W_P1A = ones([3 1]);
% P_dW_P1A = ones([3 1]);
% P_W_P2A = ones([3 1]);
% P_dW_P2A = ones([3 1]);
% A_T_d = ones([3 1]);
% A_T_f = ones([3 1]);

% A_R_B = Rx(0.1) * Ry(0.2) * Rz(0.3);
% P_R_A = Rx(0.2) * Ry(0.2) * Rz(0.3);
% J_P = diag([1 2 3]);
% J_A = diag([3 2 1]);
% J_B = diag([1 2 2]);
% B_W_B = ones([3 1]);
% B_dW_B = ones([3 1]);
% A_W_AB = ones([3 1]);
% A_dW_AB = ones([3 1]);
% P_W_P1A = ones([3 1]);
% P_dW_P1A = ones([3 1]);
% P_W_P2A = ones([3 1]);
% P_dW_P2A = ones([3 1]);
% A_T_d = ones([3 1]);
% A_T_f = ones([3 1]);

A_R_B = Rx(0.1) * Ry(0.6) * Rz(0.3);
P_R_A = Rz(0.3);
J_P = diag([1 2 3]);
J_A = diag([3 2 1]);
J_B = diag([1 2 2]);
B_W_B = [1 2 3]';
B_dW_B = [1 1 3]';
A_W_AB = [2 2 3]';
A_dW_AB = [1 1 3]';
P_W_P1A = [0 0 3]';
P_dW_P1A = [0 0 5]';
P_W_P2A = [0 0 4]';
P_dW_P2A = [0 0 1]';
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
B_T_A = A_R_B' * A_T_A;
% B_T_B = A_R_B' * A_T_A + J_B * B_dW_B + cross(B_W_B, J_B * B_W_B);
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

%% Simplified 2
alpha = B_R_A * A_W_AB;
gamma1 = alpha + B_R_P * P_W_P1A;
gamma2 = alpha + B_R_P * P_W_P2A;
A_J_P = P_R_A' * J_P * P_R_A;

J_total = B_J_A + 2*B_J_P + J_B;

kA2 = (2*B_J_P + B_J_A) * B_R_A * A_dW_AB ...
   + B_J_P * B_R_P * P_dW_P1A ...
   + B_J_P * B_R_P * P_dW_P2A;

kDelta = cross(B_W_B, (J_total)*B_W_B);

kG1 = cross(gamma1, B_J_P * B_W_B) + cross(B_W_B, B_J_P * gamma1) + cross(gamma1, B_J_P*gamma1) ...
   + cross(gamma2, B_J_P * B_W_B) + cross(B_W_B, B_J_P * gamma2) + cross(gamma2, B_J_P*gamma2) ...   
   + cross(alpha, B_J_A * B_W_B) + cross(B_W_B, B_J_A * alpha) + cross(alpha, B_J_A*alpha); 

kG2 = B_J_P * cross(B_W_B, gamma1) ...
   + B_J_P * cross(B_W_B, gamma2) ...
   + B_J_A * cross(B_W_B, alpha);

kG3 = B_R_A * A_J_P * cross(A_W_AB, P_R_A' * P_W_P1A) ...
   + B_R_A * A_J_P * cross(A_W_AB, P_R_A' * P_W_P2A);

kG = kG1 + kG2 + kG3;
% M_B3 = J_total \ (B_T -(kA2 + kG + kDelta-cross(B_W_B, J_B * B_W_B)) - cross(B_W_B, J_total * B_W_B));
M_B3 = J_B \ (B_T -(kA2 + kG + (2*B_J_P + B_J_A) * B_dW_B + kDelta-cross(B_W_B, J_B * B_W_B)) - cross(B_W_B, J_B * B_W_B));
M_B3 = J_B \ (B_T -(kA2 + kG + (2*B_J_P + B_J_A) * B_dW_B) - cross(B_W_B, J_total * B_W_B));
M_B3 = J_total \ (B_T -(kA2 + kG) - cross(B_W_B, J_total * B_W_B));

B_T_A3 = kA2 + kG + kDelta + (2*B_J_P + B_J_A) * B_dW_B;
fprintf("Method 3\n")
B_T_A2_2 = B_T_A2 + cross(B_W_B, J_B * B_W_B);
norm(B_T_A3 - B_T_A2_2)
norm(M_B - M_B3)


%% Simplified 4
% Delta Omega P

tau_adv = (2*B_J_P + B_J_A) * B_R_A * A_dW_AB ...
   + B_R_A * A_J_P * P_dW_P1A ...
   + B_R_A * A_J_P * P_dW_P2A;

DWP = P_W_P1A + P_W_P2A;
tau_gyro1 = cross(B_R_A*DWP, B_J_P*(B_W_B+alpha)) ...
   + cross(B_W_B+alpha, B_J_P * B_R_A*DWP);
tau_gyro2 = B_J_P * cross(B_W_B, B_R_A*DWP) ...
   + B_R_A * A_J_P * cross(A_W_AB, DWP);

tau_gyro = tau_gyro1 + tau_gyro2;

% tau_extra = cross(B_W_B + alpha, (B_J_A+2*B_J_P) * (B_W_B + alpha)) ...
%    + (B_J_A + 2*B_J_P) * cross(B_W_B, alpha);
tau_extra = (B_J_A + 2*B_J_P) * cross(B_W_B, alpha) ... 
   + cross(B_W_B+alpha, (B_J_A+2*B_J_P) * (B_W_B+alpha));

M_B4 = J_total \ (B_T - tau_adv - tau_gyro - tau_extra - cross(B_W_B, J_B * B_W_B));
fprintf("Method 4\n")
norm(tau_gyro2 + tau_extra - kB1)
norm(tau_gyro1 + cross(B_W_B+alpha, (B_J_A + 2*B_J_P) * (B_W_B+alpha)) - kC1)
norm(M_B3 - M_B4)

% norm(kG2+kG3 - (kB1))
% norm(kDelta+kG1 - (kC1 + cross(B_W_B, J_B * B_W_B)))

% M_B2 = (J_B) \ ...
%     ( - cross(B_W_B, (J_B + B_J_A + B_J_P) * B_W_B) + cross(B1, (B_J_A + B_J_P) * B1) ...
%       + cross(B_W_B, B_J_P * (P_R_B' * P_W_PA)) ...
%       + (B_J_A + B_J_P) * (B_R_A * A_dW_AB) ...
%       + cross(B_R_A * A_W_AB, B_J_P * (B_R_P * P_W_PA)) ...
%       + B_R_P * P_T_d );

% M_B2 - M_B
%% 


% A = (J_B + B_J_A + B_J_P) * B_dW_B ...
%   + (B_J_A + B_J_P) * cross(B_W_B, A_R_B' * A_W_AB) ...
%   + B_J_P * cross(B_W_B, P_R_B' * P_W_PA) ...
%   + (B_J_A + B_J_P) * A_R_B' * A_dW_AB ... 
%   + B_J_P * P_R_B' *P_dW_PA ...
%   + B_J_P * cross(A_R_B' * A_W_AB, P_R_B' * P_W_PA);

% % B, method 1
% B1 = B_W_B + A_R_B' * A_W_AB;
% B2 = B_W_B + A_R_B' * A_W_AB + P_R_B' * P_W_PA;

% B = cross(B_W_B, J_B * B_W_B) ...
%   + cross(B1, B_J_A * B1) ...
%   + cross(B2, B_J_P * B2);

% B, method 2
% alpha = A_R_B' * A_W_AB;
% gamma = P_R_B' * P_W_PA;
% B = cross(B_W_B, (J_B + B_J_A + B_J_P) * B_W_B) ...
%   + cross(B_W_B, (B_J_A + B_J_P) * alpha) + cross(alpha, (B_J_A + B_J_P) * B_W_B) ...
%   + cross(gamma, B_J_P * (alpha + B_W_B)) + cross((alpha + B_W_B), B_J_P * gamma) ...
%   + cross(alpha, (B_J_A + B_J_P) * alpha) + cross(gamma, B_J_P * gamma);

% cross(B_W_B, (B_J_A + B_J_P) * alpha) + cross(alpha, (B_J_A + B_J_P) * B_W_B)
% cross(gamma, B_J_P * (alpha + B_W_B)) + cross((alpha + B_W_B), B_J_P * gamma)
% B, method 3
% B = cross(B_W_B, (J_B + B_J_A + B_J_P) * B_W_B) ...
%   + cross(alpha + B_W_B, (B_J_A + B_J_P) * (alpha + B_W_B)) ...
%   + cross(alpha + B_W_B + gamma, B_J_P * (alpha + B_W_B + gamma)) ...
%   - cross(B_W_B, (B_J_A + B_J_P) * B_W_B) ...
%   - cross(alpha + B_W_B, B_J_P * (alpha + B_W_B));

% C = P_R_B' * P_T_d;

% B_T_B2 = A + B + C;

% norm(B_T_B - B_T_B2)