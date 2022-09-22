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
P = conf('pos');
psi = conf('psi');
sigma_a = conf('sigma_a');
sigma_b = conf('sigma_b');
f_max = conf('f_max');
n = length(psi);

%% Input setup
n_sample = 20;
t = linspace(0, 2 * pi, n_sample);
dt = t(2) - t(1);
f_amp = 0.5;
m_amp = 0.5;
u = [5 + f_amp * sin(t); f_amp * sin(t + pi / 3); f_amp * sin(t + 2 * pi / 3) + 10;
    m_amp * sin(t); m_amp * sin(t + pi / 3); m_amp * sin(t + 2 * pi / 3)];

% Weights for agents
W = ones(length(psi));

%% Control allocation
vecs = zeros(6, n_sample);
te = zeros([n_sample 1]);
ef = zeros([n_sample 1]);
em = zeros([n_sample 1]);

% Initial conditions
f0 = ones([10 1]) * 1;
a0 = zeros([10 1]);
b0 = zeros([10 1]);

Fs = zeros([n n_sample]);
as = zeros([n n_sample]);
bs = zeros([n n_sample]);

for i = 1:n_sample
    %[eta, xi, R, F] = allocator_interior_point(u(:, i), W, conf);
    [eta, xi, F] = allocator_nullspace(u(:, i), conf, f0, a0, b0, dt);
    vecs(:, i) = full_dof_mixing(P, psi, eta, xi, F);

    %% evaluation
    te(i) = thrust_efficiency(eta, xi, F);
    [ef(i), em(i)] = output_error(vecs(:, i), u(:, i));

    f0 = F;
    a0 = eta;
    b0 = xi;

    Fs(:, i) = F';
    as(:, i) = eta';
    bs(:, i) = xi';
end

%% Plotters
projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\0920\\control_allocation\\';
foldername = 'test\\';
filename = 'allocation_test';
dirname = strcat(projectpath, foldername);

if not(isfolder(dirname))
    mkdir(dirname)
end

key = {'projectpath', 'foldername', 'filename'};
value = {projectpath, foldername, filename};
options = containers.Map(key, value);

plot_output_profile(t, u, vecs);
plot_error_profile(t, u, vecs);
plot_metrics(t, te, ef, em, options);
plot_constraints(conf, as, bs, Fs, dt)
plot_constraints_profile(conf, as, bs, Fs, t, dt, options)
