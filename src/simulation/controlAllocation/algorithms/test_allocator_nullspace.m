addpath('../../helper_functions')
addpath('../params')
addpath('../system_func')

close all

[key, conf] = get_conf1();
u_d = [5; 1; 10; 0.1; 0.1; 0.1];
%u_d = [0; 0; 10; 0; 0; 1];
u_d = [1; 1; 10; 0; 0; 0];
f0 = ones([10 1]) * 1;
a0 = zeros([10 1]);
b0 = zeros([10 1]);
dt = 1/100;
[a, b, F, x] = allocator_nullspace(u_d, conf, f0, a0, b0, dt);

% vec = full_dof_mixing(P, psi, a0, b0, f0)

%% Configuration
P = conf('p');
psi = conf('psi');
sigma_a = conf('sigma_a');
sigma_b = conf('sigma_b');
r_sigma_a = conf('r_sigma_a');
r_sigma_b = conf('r_sigma_b');
f_max = conf('f_max');
r_f = conf('r_f');

n = length(psi);
W = get_M(n, psi, P);

Nw = null(W);
z = rand([24 1]);
Fs = Nw * z + pseudo_inverse(W) * u_d;

%% Linearized approximated solution
df = x(1:n);
da = x(n + 1:2 * n);
db = x(2 * n + 1:3 * n);
s = x(3 * n + 1:6 * n);
z = x(6 * n + 1:end);

% base
F_base = pseudo_inverse(W) * u_d;
[F_b, a_b, b_b] = inverse_input(F_base);
D_F_b = F_b - f0;
D_a_b = a_b - a0;
D_b_b = b_b - b0;

% linear
F_l = f0 + df;
a_l = a0 + da;
b_l = b0 + db;

% projection
D_F_p = F - f0;
D_a_p = a - a0;
D_b_p = b - b0;

%% Bounds
x0 = [f0; a0; b0];
x_min = -reshape((ones([1 n]) .* [f_max; sigma_a; sigma_b])', [3 * n 1]);
x_max = reshape((ones([1 n]) .* [f_max; sigma_a; sigma_b])', [3 * n 1]);
r_x_min = -dt * reshape((ones([1 n]) .* [r_f; r_sigma_a; r_sigma_b])', [3 * n 1]);
r_x_max = dt * reshape((ones([1 n]) .* [r_f; r_sigma_a; r_sigma_b])', [3 * n 1]);

dx_lb = max(x_min - x0, r_x_min);
dx_ub = min(x_max - x0, r_x_max);

disp("bound")
(x(1:30) - dx_lb > -1e-5)' & (dx_ub - x(1:30) > -1e-5)'

dx_lb = reshape(dx_lb, [n 3])';
dx_ub = reshape(dx_ub, [n 3])';

xx = reshape([1 1]' * (1:n) + [-0.5 0.5]', [2 * n 1]);
yy_f_lb = reshape([1 1]' * dx_lb(1, :), [2 * n 1]);
yy_f_ub = reshape([1 1]' * dx_ub(1, :), [2 * n 1]);
yy_a_lb = reshape([1 1]' * dx_lb(2, :), [2 * n 1]);
yy_a_ub = reshape([1 1]' * dx_ub(2, :), [2 * n 1]);
yy_b_lb = reshape([1 1]' * dx_lb(3, :), [2 * n 1]);
yy_b_ub = reshape([1 1]' * dx_ub(3, :), [2 * n 1]);

% plot angle limits for each (10 agents)
figure
subplot(3, 1, 1)
scatter(1:n, D_F_b); hold on
scatter(1:n, df); hold on
scatter(1:n, D_F_p); hold on
plot(xx, yy_f_ub, 'Color', '#BD0000'); hold on
plot(xx, yy_f_lb, 'Color', '#0000BD'); hold on
ylabel('$\Delta T_f$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
legend('LS', 'Linear', 'Nullspace');

subplot(3, 1, 2)
scatter(1:n, D_a_b); hold on
scatter(1:n, da); hold on
scatter(1:n, D_a_p); hold on
plot(xx, yy_a_ub, 'Color', '#BD0000'); hold on
plot(xx, yy_a_lb, 'Color', '#0000BD'); hold on
ylabel('$\Delta\alpha$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

subplot(3, 1, 3)
scatter(1:n, D_b_b); hold on
scatter(1:n, db); hold on
scatter(1:n, D_b_p); hold on
plot(xx, yy_b_ub, 'Color', '#BD0000'); hold on
plot(xx, yy_b_lb, 'Color', '#0000BD'); hold on
ylabel('$\Delta\beta$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
xlabel('Agents number', 'FontName', 'Times New Roman', 'FontSize', 12);
sgtitle('$\Delta X$', 'interpreter', 'latex')

yy_f_lb = -f_max * ones([2 * n 1]);
yy_f_ub = f_max * ones([2 * n 1]);
yy_a_lb = -sigma_a * ones([2 * n 1]);
yy_a_ub = sigma_a * ones([2 * n 1]);
yy_b_lb = -sigma_b * ones([2 * n 1]);
yy_b_ub = sigma_b * ones([2 * n 1]);

figure
subplot(3, 1, 1)
scatter(1:n, F_b); hold on
scatter(1:n, F_l); hold on
scatter(1:n, F); hold on
plot(xx, yy_f_ub, 'Color', '#BD0000');
plot(xx, yy_f_lb, 'Color', '#0000BD');
ylabel('$T_f$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
legend('LS', 'Linear', 'Nullspace');

subplot(3, 1, 2)
scatter(1:n, a_b); hold on
scatter(1:n, a_l); hold on
scatter(1:n, a); hold on
plot(xx, yy_a_ub, 'Color', '#BD0000');
plot(xx, yy_a_lb, 'Color', '#0000BD');
ylabel('$\alpha$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

subplot(3, 1, 3)
scatter(1:n, b_b); hold on
scatter(1:n, b_l); hold on
scatter(1:n, b); hold on
plot(xx, yy_b_ub, 'Color', '#BD0000');
plot(xx, yy_b_lb, 'Color', '#0000BD');
ylabel('$\beta$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
xlabel('Agents number', 'FontName', 'Times New Roman', 'FontSize', 12);
sgtitle('$X$', 'interpreter', 'latex')

vec = full_dof_mixing(P, psi, a, b, F)
vec = full_dof_mixing(P, psi, a0 + da, b0 + db, f0 + df)
