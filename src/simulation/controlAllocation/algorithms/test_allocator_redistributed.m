addpath('../../helper_functions')
addpath('../params')
addpath('../system_func')
addpath('../evaluation')

projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\1012_redistributed_alg\\';
foldername = 'test\\';
filename = 'my_model';
key = {'projectpath', 'foldername', 'filename'};
value = {projectpath, foldername, filename};
options = containers.Map(key, value);

close all
clc

%% Configuration
[key, conf] = get_conf1();
pos = conf('pos');
psi = conf('psi');

fprintf("=============================\n")
n = length(psi);

u_d = [4; 0; 4; 0.1; 0.1; 0.1];
%u_d = [0; 0; 10; 0; 0; 1];
%u_d = [2; 1; 10; 0; 0; 0];
f0 = ones([n 1]) * 1;
a0 = zeros([n 1]) + 0.2;
b0 = zeros([n 1]) + 0.2;
dt = 1/100;
dt = 1;
Fi0 = get_f(a0, b0, f0);
W = eye(n);

% [a, b, F] = allocator_redistributed_rect(u_d, conf, u0, W, dt);
[a, b, F] = allocator_redistributed_nonlinear(u_d, conf, a0, b0, f0, W, dt);
[a, b, F] = output_saturation(conf, n, a, b, F, a0, b0, f0, dt);
vec = full_dof_mixing(pos, psi, a, b, F);
fprintf("\n\nRedistributed allocation\n")
control_pretty_print(a, b, F)
vec'
te = thrust_efficiency(a, b, F);
[ef, em, df, dm] = output_error(u_d, vec);
fprintf("Thrust efficiency: %.4f\n", te)
fprintf("Force error:       %.4f\n", ef)
fprintf("Moment error:      %.4f\n", em)
fprintf("Force alignment:   %.4f\n", df)
fprintf("Moment alignment:  %.4f\n", dm)

% Moore
fprintf("\n\nMoore Penrose\n")
[a, b, F, vecs] = allocator_moore_penrose(u_d, conf);
[a, b, F] = output_saturation(conf, n, a, b, F, a0, b0, f0, dt);
vecs = full_dof_mixing(pos, psi, a, b, F);
control_pretty_print(a, b, F)
vecs'
te = thrust_efficiency(a, b, F);
[ef, em, df, dm] = output_error(u_d, vecs);
fprintf("Thrust efficiency: %.4f\n", te)
fprintf("Force error:       %.4f\n", ef)
fprintf("Moment error:      %.4f\n", em)
fprintf("Force alignment:   %.4f\n", df)
fprintf("Moment alignment:  %.4f\n", dm)

saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_state_norm.svg'));
saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_state_norm.fig'));

function control_pretty_print(a, b, f)
    fprintf("a\t\t|b\t\t|Tf\n");
    fprintf("-----------------------\n");

    for i = 1:length(a)
        fprintf("%.4f\t|%.4f\t|%.4f\n", a(i), b(i), f(i));
    end

end
