function [key, params] = get_params_lya()
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
    I_b = I_fm + I_a;

    mKp = 200;
    mKd = 20;

    Kp = 0.01;
    Kv = 0.01;
    Gamma_v = diag([0.2 0.2 0.8]);
    gamma_q = 1 * 0.01;
    gamma_theta1 = 0.2;
    gamma_theta2 = 1;
    gamma_theta3 = 1;
    k_theta = 1;
    Kq = 2 * diag([0.5 1 1]) * 0.04;
    Kw = 2 * diag([0.5 1 1]) * 0.02;

    delta_a = 5;
    delta_b = 3;

    v_w = [-1; -1; 0] * 0;
    C_d = diag([0.01 0.01 0.005]);
    I_xy = diag([1 1 0]);
    A_cs = 0.0114;
    esp_M = 0.01;

    key = {'g', 'rho', 'prop_d', 'CT_u', 'CT_l', 'CP_u', 'CP_l', 'm', 'm_a', 'm_fm', 'r_pg', 'r_fm', 'I_fm', 'I_a', 'I_b', 'mKp', 'mKd', ...
            'Kp', 'Kv', 'Kq', 'Kw', ...
            'Gamma_v', 'gamma_q', 'gamma_theta1', 'gamma_theta2', 'gamma_theta3', 'k_theta', ...
            'delta_a', 'delta_b', 'v_w', 'C_d', 'I_xy', 'A_cs', 'esp_M'};
    value = {g, rho, prop_d, CT_u, CT_l, CP_u, CP_l, m, m_a, m_fm, r_pg, r_fm, I_fm, I_a, I_b, mKp, mKd, ...
            Kp, Kv, Kq, Kw, ...
            Gamma_v, gamma_q, gamma_theta1, gamma_theta2, gamma_theta3, k_theta, ...
            delta_a, delta_b, v_w, C_d, I_xy, A_cs, esp_M};
    params = containers.Map(key, value);
end
