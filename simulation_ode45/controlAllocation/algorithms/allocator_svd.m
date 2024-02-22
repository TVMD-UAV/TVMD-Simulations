addpath('../../helper_functions')
addpath('../params')
addpath('../system_func')

close all

[key, conf] = get_conf1();
u = [0; 0; 10; 0; 0; 1];
f0 = ones([10 1]) * 1;
a0 = zeros([10 1]);
b0 = zeros([10 1]);
dt = 1/100;

allocator_svd2(u, conf, f0, a0, b0, dt)

function [a, b, F] = allocator_svd2(u_d, conf, f0, a0, b0, dt)
    P = conf('p');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    r_sigma_a = conf('sigma_a');
    r_sigma_b = conf('sigma_b');
    f_max = conf('f_max');
    r_f = conf('r_f');

    n = length(psi);
    W = get_M(n, psi, P);

    [U, S, V] = svd(W);
    V(:, 1:6)'
end
