close all;
rng('default')

conf_name = "model_A4_inc";
run('initialization/init_params_team.m')   

% num_seeds = 1000000;
num_seeds = 1000;

vec1 = monte_carlo(drone_params, num_seeds);
vec2 = est_attainable(drone_params);
vec3 = est_monte_roof(drone_params, 1000);
vecs = [vec1; vec2; vec3];

figure('Position', [700 100 500 400])
% depth = 8;
% mesh = pc2surfacemesh(ptCloudDownSampled,"poisson",depth);
% surfaceMeshShow(mesh); hold on
% pcshow(ptCloudDownSampled, 'BackgroundColor', 'white'); hold on

k = convhull(vecs(:, 1),vecs(:, 2),vecs(:, 3), 'Simplify', true);
tri = trisurf(k,vecs(:, 1),vecs(:, 2),vecs(:, 3)); hold on 
tri.FaceColor = '#0000EE';
tri.FaceAlpha = 0.4;
tri.EdgeColor = '#0000EE';
tri.EdgeAlpha = 0.2;
% plot_est_boundary(drone_params)
plot_est_boundary_elliptic_cone(drone_params, eye(3), [0 0 0]', 1)

axis equal
% title('Attainable Space Of Model-A-Con', 'FontName', 'Times New Roman', 'FontSize', 16)
xlabel('x', 'FontName', 'Times New Roman', 'FontSize', 14)
ylabel('y', 'FontName', 'Times New Roman', 'FontSize', 14)
zlabel('z', 'FontName', 'Times New Roman', 'FontSize', 14)
campos([-300 -300 200])

% vec_exact = est_attainable_exact(drone_params);
% k_exact = convhull(vec_exact(:, 1),vec_exact(:, 2),vec_exact(:, 3));
% tri_exact = trisurf(k_exact,vec_exact(:, 1),vec_exact(:, 2),vec_exact(:, 3)); hold on 
% tri_exact.FaceAlpha = 0.1;
% tri_exact.EdgeColor = 'none';

set(gcf, 'Renderer', 'painters')
set(gca, 'DataAspectRatio', [1 1 1])
% set(gcf, 'PaperUnit', 'normalized')
% set(gcf, 'PaperPosition', [0 0 1 1])

set(gcf, 'PaperPosition', [0 0 10 8])
set(gcf, 'PaperSize', [10 8])
fname = "C:\Users\NTU\Documents\Projects\Multidrone\outputs\attainable_space\a4_inc_attainable_space";
saveas(gcf, strcat(fname, '.fig'));
saveas(gcf, strcat(fname, '.epsc'));
saveas(gcf, strcat(fname, '.pdf'));
saveas(gcf, strcat(fname, '.svg'));

set(gcf, 'PaperPosition', [0 0 10 10])
set(gcf, 'PaperSize', [10 10])
campos([-300 0 20])
fname = "C:\Users\NTU\Documents\Projects\Multidrone\outputs\attainable_space\a4_inc_attainable_space_x";
saveas(gcf, strcat(fname, '.fig'));
saveas(gcf, strcat(fname, '.epsc'));
saveas(gcf, strcat(fname, '.pdf'));
saveas(gcf, strcat(fname, '.svg'));

set(gcf, 'PaperPosition', [0 0 10 8])
set(gcf, 'PaperSize', [10 8])
campos([0 -300 0])
fname = "C:\Users\NTU\Documents\Projects\Multidrone\outputs\attainable_space\a4_inc_attainable_space_y";
saveas(gcf, strcat(fname, '.fig'));
saveas(gcf, strcat(fname, '.epsc'));
saveas(gcf, strcat(fname, '.pdf'));
saveas(gcf, strcat(fname, '.svg'));


function vec = est_attainable(drone_params)
    %% Configurations
    P = drone_params.pos;
    psi = drone_params.psi;
    sigma_a = drone_params.sigma_a;
    sigma_b = drone_params.sigma_b;
    f_max = drone_params.f_max;

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

function vecs = est_monte_roof(drone_params, num_sim)
    vecs = zeros([num_sim 6]);
    
    %% Configurations
    P = drone_params.pos;
    psi = drone_params.psi;
    sigma_a = drone_params.sigma_a;
    sigma_b = drone_params.sigma_b;
    f_max = drone_params.f_max;
    
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

function vecs = monte_carlo(drone_params, num_sim)
    vecs = zeros([num_sim 6]);
    
    %% Configurations
    P = drone_params.pos;
    psi = drone_params.psi;
    sigma_a = drone_params.sigma_a;
    sigma_b = drone_params.sigma_b;
    f_max = drone_params.f_max;
    
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