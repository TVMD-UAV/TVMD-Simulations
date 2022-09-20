close all;
rng('default')
addpath('../helper_functions')
addpath('viz')
addpath('params')
addpath('system_func')

[key, conf] = get_conf1();

plot_attainable(conf)

function plot_attainable(conf)
    %% Configurations
    P = conf('p');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    n = length(psi);
    M = get_M(n, psi, P);
    n_max = 5000;
    vec = zeros(6, n_max);

    % Generating all possible
    f_set = zeros([3 9]);
    f_set(:, 1) = get_f(-sigma_a, 0, f_max);
    f_set(:, 2) = get_f(0, 0, f_max);
    f_set(:, 3) = get_f(sigma_a, 0, f_max);
    f_set(:, 4) = get_f(-sigma_a, -sigma_b, f_max);
    f_set(:, 5) = get_f(0, -sigma_b, f_max);
    f_set(:, 6) = get_f(sigma_a, -sigma_b, f_max);
    f_set(:, 7) = get_f(-sigma_a, sigma_b, f_max);
    f_set(:, 8) = get_f(0, sigma_b, f_max);
    f_set(:, 9) = get_f (sigma_a, sigma_b, f_max);

    for i = 1:n_max
        perm = randi(9, [1 n]);
        Fi = f_set(:, perm);
        Fi = reshape(Fi, [3 * n 1]);
        vec(:, i) = M * Fi;
    end

    plot_attainable(vec);
end
