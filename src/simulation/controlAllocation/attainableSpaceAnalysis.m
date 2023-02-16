close all;
rng('default')
addpath('../helper_functions')
addpath('viz')
addpath('params')
addpath('system_func')

[key, conf] = get_conf1();

vecs = monte_carlo(conf, 1000000);
vec2 = est_attainable(conf);
vecs = [vecs; vec2];
ptCloud = pointCloud(vecs(:, 1:3));
gridstep = 0.3;
ptCloudDownSampled = pcdownsample(ptCloud,"gridAverage",gridstep);

% depth = 8;
% mesh = pc2surfacemesh(ptCloudDownSampled,"poisson",depth);
% surfaceMeshShow(mesh); hold on
pcshow(ptCloudDownSampled, 'BackgroundColor', 'white'); hold on

k = convhull(vecs(:, 1),vecs(:, 2),vecs(:, 3));
tri = trisurf(k,vecs(:, 1),vecs(:, 2),vecs(:, 3));
tri.FaceAlpha = 0.1;
plot_est_boundary(conf)

function vec = est_attainable(conf)
    %% Configurations
    P = conf('pos');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    n = length(psi);
    M = get_M(n, psi, P);
    n_max = 500000;
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

    parfor i = 1:n_max
        perm = randi(9, [1 n]);
        Fi = f_set(:, perm);
        Fi = reshape(Fi, [3 * n 1]);
        vec(:, i) = M * Fi;
    end

    % plot_attainable(vec);
    vec = vec';
end

function plot_est_boundary(conf)
    view(3)
    psi = conf('psi');
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
    
    face = ones(4,1) * [0 4 5 1] + (1:4)';
    face(4, 3) = 5;
    face(4, 4) = 1;
    p = patch('Vertices',vertices,'Faces',face, 'EdgeColor','red','FaceColor','none','LineWidth',1);
    p.FaceAlpha = 0.3;
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
end

function vecs = monte_carlo(conf, num_sim)
    vecs = zeros([num_sim 6]);
    
    %% Configurations
    P = conf('pos');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');
    
    n = length(psi);
    M = get_M(n, psi, P);
    
    input_range = ones([n 1]) * [2*sigma_a 2*sigma_b f_max ];
    input_low = ones([n 1]) * [sigma_a sigma_b 0];

    parfor i=1:num_sim
        vecs(i, :) = monte_carlo_helper(n, M, input_range, input_low);
    end
end

function vec = monte_carlo_helper(n, M, input_range, input_low)
    u_raw = rand(n, 3) .* input_range - input_low;
    Fi = get_f(u_raw(:, 1), u_raw(:, 2), u_raw(:, 3));
    vec = M * Fi;
end

function pt_view(vecs)
    ptCloud = pointCloud(vecs(:, 1:3));

    gridstep = 0.3;
    ptCloudDownSampled = pcdownsample(ptCloud,"gridAverage",gridstep);

    % depth = 6;
    % mesh = pc2surfacemesh(ptCloudDownSampled,"poisson",depth);
    % surfaceMeshShow(mesh)
    % pcshow(ptCloud)
    pcshow(ptCloudDownSampled)
end