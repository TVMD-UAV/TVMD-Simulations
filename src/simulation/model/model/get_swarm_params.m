function [key, params] = get_swarm_params()
    % Environment
    g = 9.818; % gravity
    rho = 1.225; % kg/m3
    prop_d = 0.0254 * 9; % 8 inch = 20.3 cm

    CT_u = 0.020231; % upper propeller thrust coefficient
    CT_l = 0.020231; % lower propeller thrust coefficient
    CP_u = 0.0188; % upper propeller drag coefficient
    CP_l = 0.0188; % lower propeller drag coefficient

    % Drone
    m_a = 0.126; % Mass, Kg
    m_fm = 0.416;
    m = m_a + m_fm;
    r_pg = [0; 0; 0.03]; % Leverage length from c.p. to c.g.
    r_fm = [0; 0; -0.02]; % Leverage length from c.fm. to c.g.

    I_fm = [0.0031 0 0; 0 0.0064 0; 0 0 0.0040]; % Body Inertial
    I_a = [0.0005 0 0; 0 0.0002 0; 0 0 0.0003]; % Actuator Inertial

    mKp = 200;
    mKd = 20;
    pKp = 1000;

    Kp = 0.05;
    Kv = 0.2;
    Gamma_v = diag([0.2 0.2 0.8]);
    gamma_q = 1;
    gamma_theta1 = 0.2;
    gamma_theta2 = 1;
    gamma_theta3 = 1;
    k_theta = 1;
    Kq = 2 * diag([0.5 1 1]);
    Kw = 2 * diag([0.5 1 1]);

    delta_a = 5;
    delta_b = 3;

    v_w = [-1; -1; 0];
    C_d = diag([0.01 0.01 0.005]);
    I_xy = diag([1 1 0]);
    A_cs = 0.114;
    esp_M = 0.01;

    GRID_SIZE = 0.5 * sqrt(2) / 2;
    pos = [-1 1 0 -1 1 -1 1 -2 0 2;
        3 3 2 1 1 -1 -1 -2 -2 -2;
        0 0 0 0 0 0 0 0 1 0] * GRID_SIZE;
    psi = [0 0 0 0 0 0 0 0 0 0];

    I_b = zeros(3);

    for i = 1:length(pos)
        %x2 = pos(:, i).^2;
        %I_b = I_b + I_fm + m * diag([x2(2) + x2(3); x2(1) + x2(3); x2(2) + x2(1)]);
        I_b = I_b + I_fm + m * (pos(:, i)' * pos(:, i) * eye(3) - pos(:, i) * pos(:, i)');
    end

    m = m * length(pos);

    %% Constraints
    sigma_a = pi / 6;
    sigma_b = pi / 2;

    % rate of actuator
    r_sigma_a = 5 * pi / 6; % eta, x-axis
    r_sigma_b = 5 * pi / 2; % xi, y-axis

    f_max = 1.5 * g;
    r_f = 50;

    beta_allo = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
    prop_max = sqrt(beta_allo \ [f_max; 0] / (rho * prop_d^4));

    key = {'g', 'rho', 'prop_d', 'CT_u', 'CT_l', 'CP_u', 'CP_l', 'm', 'm_a', 'm_fm', 'r_pg', 'r_fm', 'I_fm', 'I_a', 'I_b', 'mKp', 'mKd', 'pKp', ...
            'Kp', 'Kv', 'Kq', 'Kw', ...
            'Gamma_v', 'gamma_q', 'gamma_theta1', 'gamma_theta2', 'gamma_theta3', 'k_theta', ...
            'delta_a', 'delta_b', 'v_w', 'C_d', 'I_xy', 'A_cs', 'esp_M', ...
            'pos', 'psi', 'sigma_a', 'sigma_b', 'r_sigma_a', 'r_sigma_b', 'f_max', 'r_f', 'prop_max'};
    value = {g, rho, prop_d, CT_u, CT_l, CP_u, CP_l, m, m_a, m_fm, r_pg, r_fm, I_fm, I_a, I_b, mKp, mKd, pKp, ...
            Kp, Kv, Kq, Kw, ...
            Gamma_v, gamma_q, gamma_theta1, gamma_theta2, gamma_theta3, k_theta, ...
            delta_a, delta_b, v_w, C_d, I_xy, A_cs, esp_M, ...
            pos, psi, sigma_a, sigma_b, r_sigma_a, r_sigma_b, f_max, r_f, prop_max};
    params = containers.Map(key, value);
end
