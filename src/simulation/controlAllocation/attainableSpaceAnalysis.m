close all;
rng('default')
addpath('../helper_functions')
addpath('viz')
addpath('params')
addpath('system_func')
addpath('../model/model')
addpath('../model/model/swarm_conf')

projectpath = 'H:\\我的雲端硬碟\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\outputs\\230208_attainable_space\\';
foldername = 'A10-Con\\';
filename = 'attainable_space';

[key, conf] = get_swarm_params('model_A10_con');
vec1 = monte_carlo(conf, 1000000);
vec2 = est_attainable(conf);
vecs = [vecs; vec2];
ptCloud = pointCloud(vecs(:, 1:3));
gridstep = 0.3;
ptCloudDownSampled = pcdownsample(ptCloud,"gridAverage",gridstep);

figure('Position', [700 100 500 400])
% depth = 8;
% mesh = pc2surfacemesh(ptCloudDownSampled,"poisson",depth);
% surfaceMeshShow(mesh); hold on
pcshow(ptCloudDownSampled, 'BackgroundColor', 'white'); hold on

k = convhull(vecs(:, 1),vecs(:, 2),vecs(:, 3));
tri = trisurf(k,vecs(:, 1),vecs(:, 2),vecs(:, 3)); hold on 
tri.FaceAlpha = 0.5;
tri.EdgeColor = 'none';
% plot_est_boundary(conf)
plot_est_boundary_elliptic_cone(conf)
axis equal
% title('Attainable Space Of Model-A-Con', 'FontName', 'Times New Roman', 'FontSize', 16)
xlabel('x', 'FontName', 'Times New Roman', 'FontSize', 14)
ylabel('y', 'FontName', 'Times New Roman', 'FontSize', 14)
zlabel('z', 'FontName', 'Times New Roman', 'FontSize', 14)
campos([-300 50 100])

% vec_exact = est_attainable_exact(conf);
% k_exact = convhull(vec_exact(:, 1),vec_exact(:, 2),vec_exact(:, 3));
% tri_exact = trisurf(k_exact,vec_exact(:, 1),vec_exact(:, 2),vec_exact(:, 3)); hold on 
% tri_exact.FaceAlpha = 0.1;
% tri_exact.EdgeColor = 'none';

set(gcf, 'Renderer', 'painters')
set(gca, 'DataAspectRatio', [1 1 1])
print(gcf, '-depsc', 'test.eps')
saveas(gcf, strcat(projectpath, foldername, filename, '.png'));
saveas(gcf, strcat(projectpath, foldername, filename, '.svg'));
saveas(gcf, strcat(projectpath, foldername, filename, '.epsc'));
saveas(gcf, strcat(projectpath, foldername, filename, '.fig'));

% region [est_attainable]
function vec = est_attainable(conf)
    %% Configurations
    P = conf('pos');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    n = length(psi);
    M = get_M(n, psi, P);
    n_max = 1000000;
    vec = zeros(n_max, 6);

    % Generating all possible
    f_set = zeros([3 9]);
    f_set(:, 1) = get_f_trimmed(-sigma_a,        0, f_max);
    f_set(:, 2) = get_f_trimmed(       0,        0, f_max);
    f_set(:, 3) = get_f_trimmed( sigma_a,        0, f_max);
    f_set(:, 4) = get_f_trimmed(-sigma_a, -sigma_b, f_max);
    f_set(:, 5) = get_f_trimmed(       0, -sigma_b, f_max);
    f_set(:, 6) = get_f_trimmed( sigma_a, -sigma_b, f_max);
    f_set(:, 7) = get_f_trimmed(-sigma_a,  sigma_b, f_max);
    f_set(:, 8) = get_f_trimmed(       0,  sigma_b, f_max);
    f_set(:, 9) = get_f_trimmed (sigma_a,  sigma_b, f_max);

    parfor i = 1:n_max
        perm = randi(9, [1 n]);
        Fi = f_set(:, perm);
        Fi = reshape(Fi, [3 * n 1]);
        vec(i, :) = M * Fi;
    end
end
% end region [est_attainable]

function vec = est_attainable_exact(conf)
    %% Configurations
    P = conf('pos');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    n = length(psi);
    M = get_M(n, psi, P);
    n_max = 1000000;
    vec = zeros(n_max, 6);

    % Generating all possible
    f_set = zeros([3 9]);
    f_set(:, 1) = get_f(-sigma_a,        0, f_max);
    f_set(:, 2) = get_f(       0,        0, f_max);
    f_set(:, 3) = get_f( sigma_a,        0, f_max);
    f_set(:, 4) = get_f(-sigma_a, -sigma_b, f_max);
    f_set(:, 5) = get_f(       0, -sigma_b, f_max);
    f_set(:, 6) = get_f( sigma_a, -sigma_b, f_max);
    f_set(:, 7) = get_f(-sigma_a,  sigma_b, f_max);
    f_set(:, 8) = get_f(       0,  sigma_b, f_max);
    f_set(:, 9) = get_f (sigma_a,  sigma_b, f_max);

    parfor i = 1:n_max
        perm = randi(9, [1 n]);
        Fi = f_set(:, perm);
        Fi = reshape(Fi, [3 * n 1]);
        vec(i, :) = M * Fi;
    end
end

function vecs = est_monte_roof(conf, num_sim)
    vecs = zeros([num_sim 6]);
    
    %% Configurations
    P = conf('pos');
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');
    
    n = length(psi);
    M = get_M(n, psi, P);
    
    input_range = [2*sigma_a 2*sigma_b 0];
    input_low = [sigma_a sigma_b -f_max];

    parfor i=1:num_sim
        u_raw = rand(1, 3) .* input_range - input_low;
        Fi = get_f_trimmed(ones([n 1]) * u_raw(1), ones([n 1]) * u_raw(2), ones([n 1]) * u_raw(3));
        vecs(i, :) = M * Fi;
    end
end

function plot_est_boundary_elliptic_cone(conf)
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');
    n = length(psi);
    n_x = sum(psi == 0);
    n_y = n - n_x;

    % c_x = @(tf) fun_g(n_x, sigma_b, tf, f_max) + fun_g(n_y, sigma_a, tf, f_max);
    % c_y = @(tf) fun_g(n_x, sigma_a, tf, f_max) + fun_g(n_y, sigma_b, tf, f_max);

    % funcx_r = @(phi, tf) c_x(tf) .* cos(phi); 
    % funcy_r = @(phi, tf) c_y(tf) .* sin(phi); 
    % funcz_r = @(phi, tf) sqrt(tf.^2 - (funcx_r(phi, tf)).^2 - (funcy_r(phi, tf)).^2); 
    % fsurf(funcx_r, funcy_r, funcz_r, [0 2*pi 0 n*f_max], 'FaceColor', '#77AC30', 'FaceAlpha', 0.2, 'EdgeColor', 'none'); hold on 

    % c_x = @(z) fun_g2(n_x, sigma_b, z, f_max) + fun_g2(n_y, sigma_a, z, f_max);
    % c_y = @(z) fun_g2(n_x, sigma_a, z, f_max) + fun_g2(n_y, sigma_b, z, f_max);

    c_x = @(z) fun_g2(n_x, sigma_b, z, f_max) + fun_g2(n_y, sigma_a, z, f_max);
    c_y = @(z) fun_g2(n_x, sigma_a, z, f_max) + fun_g2(n_y, sigma_b, z, f_max);
    % c_x = @(z) n_x * f_max + (n_y>0) * z *tan(sigma_a); 
    % c_y = @(z) n_y * f_max + (n_x>0) * z *tan(sigma_a); 
    % i = 80:2:80;
    i = 1:2:100;
    plot_ellipse(c_x(i), c_y(i), i, n*f_max)
    i = 80:2:n*f_max;
    % plot_roof(c_x(i), c_y(i), i, n*f_max)
end

function plot_ellipse(a, b, z, tf)
    phi = linspace(0, 2*pi, 500)';
    % r = sqrt(tf.^2 - z.^2);
    % xe = r .* cos(phi);
    % ye = r .* sin(phi);
    xa = a .* cos(phi);
    yb = b .* sin(phi);
    z = 0 * phi + z;
    % plot3(xe, ye, z, 'Color', [0, 1, 0, 0.2]); hold on 
    % plot3(xa, yb, z, 'Color', [0, 0, 1, 0.2]); hold on 
    % mask = ((xa.^2 + yb.^2) > (xe.^2 + ye.^2));
    % xe(mask) = xa(mask);
    % ye(mask) = yb(mask);
    z(sqrt(xa.^2 + yb.^2 + z.^2) > tf) = inf;
    plot3(xa, yb, z, 'Color', [1, 0, 0, 0.5]); hold on 
end

function plot_roof(a, b, z, tf)
    phi = linspace(0, 2*pi)';
    % phi2 = atan(a ./ b);
    r = sqrt(tf.^2 - z.^2);
    x = r .* cos(phi);
    y = r .* sin(phi);
    z = 0 * phi + z;
    % xa = a .* cos(phi);
    % yb = b .* sin(phi);
    % size(ones([1 length(z)]).*abs(sin(phi)))
    % size(ones([length(phi) 1]).*sqrt(abs((r.^2 - b.^2) ./ (a.^2 - b.^2))))
    z(ones([1 length(a)]).*abs(sin(phi)) > ones([length(phi) 1]).*sqrt(abs((r.^2 - b.^2) ./ (a.^2 - b.^2)))) = inf;
    plot3(x, y, z, 'Color', [1, 0, 0, 0.3]); hold on 
    % plot3(xa, yb, z, 'Color', [0, 1, 0, 0.3]); hold on 
end

function gk = fun_g(n_k, sigma_k_bar, tf, f_max)
    if (sigma_k_bar >= pi / 2)
        gk = n_k * f_max;
    else
        gk = (n_k > 0) * tf * sin(sigma_k_bar);
    end
end

function gk = fun_g2(n_k, sigma_k_bar, z, f_max)
    if (sigma_k_bar >= pi / 2)
        gk = n_k * f_max;
    else
        gk = (n_k > 0) * z * tan(sigma_k_bar);
        % gk = n_k * tan(sigma_k) * f_max;
    end
end

function plot_est_boundary(conf)
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
    p = patch('Vertices',vertices,'Faces',face, 'EdgeColor','red','FaceColor','none','LineWidth',1); hold on
    p.FaceAlpha = 0.3;
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
    
    % input_range = ones([n 1]) * [2*sigma_a 2*sigma_b f_max ];
    % input_low = ones([n 1]) * [sigma_a sigma_b 0];
    input_range = ones([n 1]) * [2*sigma_a 2*sigma_b 0];
    input_low = ones([n 1]) * [sigma_a sigma_b -f_max];

    parfor i=1:num_sim
        vecs(i, :) = monte_carlo_helper(n, M, input_range, input_low);
    end
end

function vec = monte_carlo_helper(n, M, input_range, input_low)
    u_raw = rand(n, 3) .* input_range - input_low;
    Fi = get_f_trimmed(u_raw(:, 1), u_raw(:, 2), u_raw(:, 3));
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

% The trimmed f
function Fi = get_f_trimmed(a, b, tf)

    if length(a) == 1
        Fi = [(sin(b) .* tf);
            -(sin(a) .* cos(b).* tf);
            (cos(b) .* cos(a) .* tf)];
    else
        Fi = [(sin(b') .* tf');
            -(sin(a') .* cos(b').* tf');
            (cos(b') .* cos(a') .* tf')];
        n = length(a);
        Fi = reshape(Fi, [3 * n 1]);
    end

end