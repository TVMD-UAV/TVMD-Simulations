addpath("../params")

[key, conf] = get_conf1();

n_sample = 100;
P = conf('pos');
psi = conf('psi');
n = length(psi);

t = linspace(0, 2*pi, n_sample);

for i=1:n_sample
    a = ones([n 1]) * cos(t(i));
    b = ones([n 1]) * sin(t(i));
    tf = ones([n 1]) * sin(t(i)) + 1;
    f = get_f(a, b, tf);
    [f1, a1, b1] = inverse_input(f);
    norm(f1 - tf) + norm(a - a1) + norm(b1 - b)
    full_dof_mixing(P, psi, a, b, tf)
end