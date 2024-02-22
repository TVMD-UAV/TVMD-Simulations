close all;
rng('default')
addpath('../helper_functions')
addpath('viz')
addpath('algorithms')
addpath('controller')
addpath('evaluation')
addpath('params')
addpath('system_func')
addpath('../model/model')
addpath('../model/model/swarm_conf')

projectpath = 'H:\\我的雲端硬碟\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\outputs\\230210_fullpose_conf_seq\\';
foldername = 'A10-Inc\\interior\\';
% foldername = 'test\\';
filename = 'allocation_seq';

%% Configurations
% [key, conf] = get_conf1();
[key, conf] = get_swarm_params('model_A10_inc');
P = conf('pos');
psi = conf('psi');
g = conf('g');
sigma_a = conf('sigma_a');
sigma_b = conf('sigma_b');
f_max = conf('f_max');
n = length(psi);

%% Input setup
n_sample = 100;
t = linspace(0, 2 * pi, n_sample);
dt = t(2) - t(1);
f_amp = 5*g;
m_amp = 1*g;
k = 3;
l = 0.2;
u = [5*g + f_amp * sin(l * t.^k); f_amp * sin(l * t.^k + pi / 3); 0 * sin(l * t.^k + 2 * pi / 3) + 10*g;
    m_amp * sin(l * t.^k); m_amp * sin(l * t.^k + pi / 3); m_amp * sin(l * t.^k + 2 * pi / 3)];

% Weights for agents
W = ones(length(psi));

%% Control allocation
vecs = zeros(6, n_sample);
te = zeros([n_sample 1]);
ef = zeros([n_sample 1]);
em = zeros([n_sample 1]);
df = zeros([n_sample 1]);
dm = zeros([n_sample 1]);

% Initial conditions
f0 = ones([10 1]) * 1;
a0 = zeros([10 1]);
b0 = zeros([10 1]);

Fs = zeros([n n_sample]);
as = zeros([n n_sample]);
bs = zeros([n n_sample]);

u_d = u;

for i = 1:n_sample
    % u_d(:, i) = force_sat(conf, u(:, i));
    % [eta, xi, F, v] = allocator_moore_penrose(u_d(:, i), conf);
    [eta, xi, R, F] = allocator_interior_point(u_d(:, i), W, conf, a0, b0, f0);

    % [al0, bl0, fl0] = allocator_moore_penrose(u(:, i), conf);
    % [al0, bl0, fl0] = output_saturation2(conf, n, al0, bl0, fl0);
    % [eta, xi, F] = allocator_nullspace(u(:, i), conf, fl0, al0, bl0, dt);
    % [eta, xi, F] = allocator_nullspace(u(:, i), conf, f0, a0, b0, dt);
    
    % [eta, xi, F] = allocator_redistributed_nonlinear(u_d(:, i), conf, a0, b0, f0, W, dt);
    % [eta, xi, F] = allocator_null_redistr(u_d(:, i), conf, a0, b0, f0, W, dt);
    % [eta, xi, F] = allocator_null_redistr_moment_enhance(u_d(:, i), conf, a0, b0, f0, W, dt);
    % [eta, xi, F] = allocator_null_redistr_torque(u_d(:, i), conf, a0, b0, f0, W, dt);

    [eta, xi, F] = output_saturation(conf, n, eta, xi, F, a0, b0, f0, dt);
    vecs(:, i) = full_dof_mixing(P, psi, eta, xi, F);

    %% evaluation
    te(i) = thrust_efficiency(eta, xi, F);
    [ef(i), em(i), df(i), dm(i)] = output_error(vecs(:, i), u_d(:, i));

    f0 = F;
    a0 = eta;
    b0 = xi;

    Fs(:, i) = F;
    as(:, i) = eta;
    bs(:, i) = xi;
end

%% Plotters
dirname = strcat(projectpath, foldername);

if not(isfolder(dirname))
    mkdir(dirname)
end

matfilename = strcat(projectpath, foldername, filename);
save(matfilename, 'conf', 't', 'u', 'vecs', 'te', 'ef', 'em', 'df', 'dm', 'as', 'bs', 'Fs');

key = {'projectpath', 'foldername', 'filename'};
value = {projectpath, foldername, filename};
options = containers.Map(key, value);

% plot_output_profile(t, u, vecs, options);
plot_output_profile2(t, u, u_d, vecs, options);
plot_error_profile(t, u, vecs, options);
plot_metrics(t, te, ef, em, df, dm, options);
% plot_constraints(conf, as, bs, Fs, dt)
plot_constraints_profile(conf, as', bs', Fs', t', dt, options);

figure
cmap = parula(n_sample);
for i=1:n_sample
    quiver3(0, 0, 0, u(1, i), u(2, i), u(3, i), 'Color', cmap(i, :), 'LineStyle', ':'); hold on
    quiver3(0, 0, 0, u_d(1, i), u_d(2, i), u_d(3, i), 'Color', cmap(i, :), 'LineStyle', '-'); hold on
end
plot_est_boundary(conf, eye(3));

function plot_output_profile2(t, u, u_d, vecs, options)
    figure('Position', [10 10 800 800])
    subplot(6, 1, 1);
    plot(t, u(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD', 'marker', 'o'); hold on
    plot(t, vecs(1, :), 'DisplayName', '$$F_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD');
    plot(t, u_d(1, :), 'DisplayName', '$$F_{xd}$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD', 'marker', '^');
    ylabel('$F_x$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    title('Forces')

    subplot(6, 1, 2);
    plot(t, u(2, :), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319', 'marker', 'o'); hold on
    plot(t, vecs(2, :), 'DisplayName', '$$F_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319');
    plot(t, u_d(2, :), 'DisplayName', '$$F_{yd}$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD', 'marker', '^');
    ylabel('$F_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(6, 1, 3);
    plot(t, u(3, :), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120', 'marker', 'o'); hold on
    plot(t, vecs(3, :), 'DisplayName', '$$F_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120');
    plot(t, u_d(3, :), 'DisplayName', '$$F_{zd}$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD', 'marker', '^');
    ylabel('$F_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(6, 1, 4);
    plot(t, u(4, :), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD', 'marker', 'o'); hold on
    plot(t, vecs(4, :), 'DisplayName', '$$M_x$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD');
    ylabel('$M_x$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    title('Moments')

    subplot(6, 1, 5);
    plot(t, u(5, :), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319', 'marker', 'o'); hold on
    plot(t, vecs(5, :), 'DisplayName', '$$M_y$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#D95319');
    ylabel('$M_y$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    subplot(6, 1, 6);
    plot(t, u(6, :), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#EDB120', 'marker', 'o'); hold on
    plot(t, vecs(6, :), 'DisplayName', '$$M_z$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#EDB120');
    ylabel('$M_z$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_output.svg'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_output.fig'));
    saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_output.eps'));
end

function plot_est_boundary(conf, R)
    psi = conf('psi');
    % g = conf('g');
    sigma_a = conf('sigma_a');
    f_max = conf('f_max');
    n = length(psi);

    n_x = sum(psi == 0);
    n_y = n - n_x;
    x_basis = @(z) f_max * n_x + (n_y ~= 0) * z * tan(sigma_a);
    y_basis = @(z) f_max * n_y + (n_x ~= 0) * z * tan(sigma_a);
    vertices = zeros([8 3]);
    i_x = [1 -1 -1 1];
    i_y = [1 1 -1 -1];
    vertices(:, 1) = [i_x * x_basis(0) i_x  * x_basis(f_max * n/2)];
    vertices(:, 2) = [i_y * y_basis(0) i_y  * y_basis(f_max * n/2)];
    vertices(:, 3) = [zeros([1 4]) ones([1 4]) * f_max * n/2];
    vertices = (R * (vertices'))';
    
    face = ones(4,1) * [0 4 5 1] + (1:4)';
    face(4, 3) = 5;
    face(4, 4) = 1;
    p = patch('Vertices',vertices,'Faces',face, 'EdgeColor','red','FaceColor','none','LineWidth',1);
    p.FaceAlpha = 0.3;
end

% region [output_saturation]
function [a, b, F] = output_saturation2(conf, n, a, b, F)
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    x_min = -reshape((ones([1 n]) .* [f_max; sigma_a; sigma_b])', [3 * n 1]);
    x_max = reshape((ones([1 n]) .* [f_max; sigma_a; sigma_b])', [3 * n 1]);

    w = min(x_max, max(x_min, [F; a; b]));
    F = w(1:n);
    a = w(n + 1:2 * n);
    b = w(2 * n + 1:3 * n);
end

% endregion [output_saturation]