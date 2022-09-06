close all;
rng('default')
addpath('../helper_functions')
addpath('viz')
addpath('algorithms')
addpath('evaluation')
addpath('params')
addpath('system_func')

%% Configurations
[key, conf] = get_conf1();
P = conf('p');
psi = conf('psi');
sigma_a = conf('sigma_a');
sigma_b = conf('sigma_b');
f_max = conf('f_max');

%% Input setup
n_sample = 10;
t = linspace(0, 2 * pi, n_sample);
f_amp = 0.1;
m_amp = 0.1;
u = [f_amp * sin(t); f_amp * sin(t + pi / 3); f_amp * sin(t + 2 * pi / 3) + 10;
    m_amp * sin(t); m_amp * sin(t + pi / 3); m_amp * sin(t + 2 * pi / 3)];

% Weights for agents
W = ones(length(psi));

%% Control allocation
vecs = zeros(6, length(psi));
te = zeros([n_sample 1]);
ef = zeros([n_sample 1]);
em = zeros([n_sample 1]);

for i = 1:n_sample
    [eta, xi, R, F] = allocator_interior_point(u(:, i), W, conf);
    vecs(:, i) = full_dof_mixing(P, psi, eta, xi, F);

    %% evaluation
    te(i) = thrust_efficiency(eta, xi, F);
    [ef(i), em(i)] = output_error(vecs(:, i), u(:, i));
end

%% Plotters
plot_forces_profile(t, u, vecs);
plot_moments_profile(t, u, vecs);
plot_metrics(t, te, ef, em);
