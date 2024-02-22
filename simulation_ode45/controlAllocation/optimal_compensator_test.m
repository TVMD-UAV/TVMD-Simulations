close all;
% clear all;
rng('default')

addpath('../model/model')
addpath("../model/model/swarm_conf")

[key, params] = get_swarm_params("model_A9_inc");

mKp = params('mKp');
mKd = params('mKd');
mKp = params('mKp');

w_n = sqrt(mKp);
zeta = mKd / (2 * w_n);
k = mKp;

A = [0 1; -w_n^2 -2*zeta*w_n];
B = [0; k];
C = eye(2);
% C = [1; 0];
D = 0;

Ts = 0.01 / 2;
sysd = c2d(ss(A, B, C, D), Ts);
% Q = diag([1 1]);
Q = diag([10 0.001 0.001 0.001]);
% Q = diag([1 1 1 1]);
R = 0.001;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F = [sysd.A sysd.A-eye(2); zeros(2) eye(2)];
G = [sysd.B; zeros(2, 1)];
S = zeros([4 4 length(t)]);
V = zeros([1 length(t)]);

for i=1:length(t)
    S(:, :, i) = Q;
end
T = 3; 
for i=1:T-1
    k = T-i;
    St1 = squeeze(S(:, :, k+1));
    S(:, :, k) = F' * (St1 - St1 * G / (G' * St1 * G + R) * G' * St1) * F + Q;
end

X = zeros([4 length(t)]);
u = zeros([1 length(t)]);
x0 = [2-10; 0; 10; 0];
X(:, 1) = x0;
V(1) = x0' * (S(:, :, 1) - Q) * x0;
% xt = [10; 0];

for i=1:length(t)-1
    St1 = squeeze(S(:, :, i+1));
    St1
    K = -(G' * St1 * G + R) \ G' * St1 * F
    u(i) = K * X(:, i);
    X(:, i+1) = F * X(:, i) + G * u(i);
    V(i+1) = V(i) + u(i) * R * u(i) + X(:, i+1)' * Q * X(:, i+1);
end
% u = max(0,min(t-1,1));
% lsim(sysd, zeros([1 length(t)]), t, [2; 0]);
scatter(t, X(1, :) + x0(3), 'DisplayName', 'X(t)'); hold on 
scatter(t, u, 'DisplayName', 'u(t)'); hold on 
xlabel('Time (s)')
ylabel('Value')
title('Response Profile')
legend

figure 
scatter(t, V, 'DisplayName', 'V(t)'); hold on 
xlabel('Performance Index')
ylabel('Value')
title('Performance Index Profile')
legend
return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 

% [S,K,L] = idare(sysd.A, sysd.B, Q, R);
Q = diag([10 0.001]);
t = 0:Ts:10*Ts;  
x0 = [2; 0];
xt = [10; 0];
bt = zeros(2, length(t));

% Steps to track the target
T = 5; 

X = zeros([2 length(t)]);
S = zeros([2 2 length(t)]);
u = zeros([1 length(t)]);
% Solve for b[t]
for i=1:length(t)
    S(:, :, i) = Q;
end
F = sysd.A;
G = sysd.B;

for i=1:T-1
    k = T-i;
    St1 = squeeze(S(:, :, k+1));
    S(:, :, k) = F' * (St1 - St1 * G / (G' * St1 * G + R) * G' * St1) * F + Q;
end
for i=1:T-1
    k = T-i;
    St1 = squeeze(S(:, :, k+1));
    K = -(G' * St1 * G + R) \ G' * St1 * F;
    bt(:, k) = (F' + K' * G') * bt(:, k+1) - Q * xt;
end
X(:, 1) = x0;

for i=1:length(t)-1
    
    St1 = squeeze(S(:, :, i+1));
    % (G' * St1 * G + R) \ G'
    % St1 * F * X(:, i)
    % (St1 * F * X(:, i) + bt(:, i+1))
    i
    inv(G' * St1 * G + R) * G'
    -(G' * St1 * G + R) \ G' * (St1 * F * X(:, i)) 
    -(G' * St1 * G + R) \ G' * (bt(:, i+1))
    u(i) = -(G' * St1 * G + R) \ G' * (St1 * F * X(:, i) + bt(:, i+1));
    X(:, i+1) = F * X(:, i) + G * u(i);
end
% u = max(0,min(t-1,1));
% lsim(sysd, zeros([1 length(t)]), t, [2; 0]);
scatter(t, X(1, :), 'DisplayName', 'X(t)'); hold on 
scatter(t, bt(1, :), 'DisplayName', 'b1(t)'); hold on 
scatter(t, bt(2, :), 'DisplayName', 'b2(t)'); hold on 
scatter(t, u, 'DisplayName', 'u(t)'); hold on 
legend

figure
scatter(t, squeeze(S(1, 1, :)), 'DisplayName', 'S11(t)'); hold on 
scatter(t, squeeze(S(1, 2, :)), 'DisplayName', 'S12(t)'); hold on 
scatter(t, squeeze(S(2, 1, :)), 'DisplayName', 'S21(t)'); hold on 
scatter(t, squeeze(S(2, 2, :)), 'DisplayName', 'S22(t)'); hold on 
legend

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% w_n = sqrt(mKp);
% zeta = mKd / (2 * w_n);
% k = mKp;

% w_d = w_n * sqrt(1 - zeta^2);
% sigma = -zeta * w_n;

% C1 = (w_n / w_d) * exp(sigma * dt) * sin(w_d * dt + atan(w_d / -sigma));
% C2 = (exp(sigma * dt) / w_d) * sin(w_d * dt);
% C3 = (k / w_d) * (w_d + exp(sigma * dt) * (sigma*sin(w_d*dt) - w_d*cos(w_d*dt))) / w_n^2;
% M = (delta_z_di + (1-C3-C1)*x_zi - C2*v_zi) ./ (C3 * delta_z_di);

