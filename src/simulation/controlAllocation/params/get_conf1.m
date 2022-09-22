function [key, params] = get_conf1()
    % Configuration
    GRID_SIZE = 0.3 * sqrt(2) / 2;
    p = [3 3 2 1 1 -1 -1 -2 -2 -2;
        -1 1 0 -1 1 -1 1 -2 0 2;
        0 0 0 0 0 0 0 0 -1 0] * GRID_SIZE;
    psi = [0 0 0 0 0 0 0 0 0 0];

    sigma_a = pi / 6;
    sigma_b = pi / 2;

    % rate of actuator
    r_sigma_a = 1 * pi / 6; % eta, x-axis
    r_sigma_b = 1 * pi / 2; % xi, y-axis

    f_max = 1.5;
    r_f = 10;

    key = {'pos', 'psi', 'sigma_a', 'sigma_b', 'r_sigma_a', 'r_sigma_b', 'f_max', 'r_f'};
    value = {p, psi, sigma_a, sigma_b, r_sigma_a, r_sigma_b, f_max, r_f};
    params = containers.Map(key, value);
end
