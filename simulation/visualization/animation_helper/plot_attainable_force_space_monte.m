function plot_attainable_force_space_monte(drone_params, num_seeds, scaling)
    vec1 = monte_carlo(drone_params, num_seeds);
    vec2 = est_attainable(drone_params);
    vec3 = est_monte_roof(drone_params, 10000);
    vecs = [vec1; vec2; vec3] * scaling;

    k = convhull(vecs(:, 1),vecs(:, 2),vecs(:, 3));
    % tri = trisurf(k,vecs(:, 1),vecs(:, 2),vecs(:, 3)); hold on 
    tri = trisurf(k,vecs(:, 1),vecs(:, 2),vecs(:, 3), "FaceColor", "red"); hold on 
    tri.FaceAlpha = 0.2;
    tri.EdgeColor = 'black';
    tri.EdgeAlpha = 0.2;
    % tri.EdgeColor = 'none';
    
    axis equal
    set(gcf, 'Renderer', 'painters')
    set(gca, 'DataAspectRatio', [1 1 1])
end

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