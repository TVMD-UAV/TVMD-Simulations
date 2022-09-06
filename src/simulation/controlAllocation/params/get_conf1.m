function [key, params] = get_conf1()
    % Configuration
    GRID_SIZE = 0.3 * sqrt(2) / 2;
    p = [3 3 2 1 1 -1 -1 -2 -2 -2;
        -1 1 0 -1 1 -1 1 -2 0 2;
        0 0 0 0 0 0 0 0 -1 0] * GRID_SIZE;
    psi = [0 0 0 0 0 0 0 0 0 0];

    sigma_a = pi / 6;
    sigma_b = pi / 2;

    f_max = 1.5;

    key = {'p', 'psi', 'sigma_a', 'sigma_b', 'f_max'};
    value = {p, psi, sigma_a, sigma_b, f_max};
    params = containers.Map(key, value);
end
