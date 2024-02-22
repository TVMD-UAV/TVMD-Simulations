function [key, params] = get_birotor_params()
    % Environment
    g = 9.818; % gravity
    rho = 1.225; % kg/m3
    prop_d = 0.0254 * 9; % 8 inch = 20.3 cm

    CT_u = 0.020231; % upper propeller thrust coefficient
    CT_l = 0.020231; % lower propeller thrust coefficient
    CP_u = 0.0188; % upper propeller drag coefficient
    CP_l = 0.0188; % lower propeller drag coefficient

    % Drone
    m = 0.542; % Mass, Kg
    %I_b = diag([0.0061 0.0064 0.0040]); % Body Inertial
    I_b = diag([0.00033 0.00736 0.00755]); % Body Inertial
    I_a = diag([0.0001 0.0001 0.0002]); % Actuator Inertial
    I_r = diag([0.00006 0.00006 0.000001]); % Propeller Inertial

    l = 0.3;
    l = 0.185;
    h = 0.03;
    h = 0.05;

    mKp = 200;
    mKd = 20;

    key = {'g', 'rho', 'prop_d', 'CT_u', 'CT_l', 'CP_u', 'CP_l', 'm', 'l', 'h', 'I_b', 'I_r', 'I_a', 'mKp', 'mKd'};
    value = {g, rho, prop_d, CT_u, CT_l, CP_u, CP_l, m, l, h, I_b, I_r, I_a, mKp, mKd};
    params = containers.Map(key, value);
end
