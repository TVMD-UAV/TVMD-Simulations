function [key, params] = get_params()
    % Environment
    g = 9.818;     % gravity
    rho = 1.225;   % kg/m3
    prop_d = 0.0254*9; % 8 inch = 20.3 cm

    CT_u = 0.020231; % upper propeller thrust coefficient
    CT_l = 0.020231; % lower propeller thrust coefficient
    CP_u = 0.0188;   % upper propeller drag coefficient
    CP_l = 0.0188;   % lower propeller drag coefficient
    
    % Drone
    m_a = 0.126;     % Mass, Kg
    m_fm = 0.416;
    r_pg = [0; 0; 0.03];  % Leverage length from c.p. to c.g.
    r_fm = [0; 0; -0.02]; % Leverage length from c.fm. to c.g. 

    I_fm = [0.0031 0 0; 0 0.0064 0; 0 0 0.0040]; % Body Inertial
    I_a  = [0.0005 0 0; 0 0.0002 0; 0 0 0.0003]; % Actuator Inertial

    key = {'g', 'rho', 'prop_d', 'CT_u', 'CT_l', 'CP_u', 'CP_l', 'm_a', 'm_fm', 'r_pg', 'r_fm', 'I_fm', 'I_a'};
    value = {g, rho, prop_d, CT_u, CT_l, CP_u, CP_l, m_a, m_fm, r_pg, r_fm, I_fm, I_a};
    params = containers.Map(key, value);
end