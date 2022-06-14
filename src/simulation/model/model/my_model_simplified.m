function [dydt, commands, meta] = my_model_simplified(params, u, y)
    %% Parameters
    g = params('g');     % gravity
    
    % Drone
    m = params('m');
    m_a = params('m_a');     % Mass, Kg
    m_fm = params('m_fm');
    r_pg = params('r_pg');  % Leverage length from c.p. to c.g.
    r_fm = params('r_fm'); % Leverage length from c.fm. to c.g. 

    I_fm = params('I_fm'); % Body Inertial
    I_a  = params('I_a'); % Actuator Inertial

    %% State variables
    W = y(1:3);
    Q = y(4:7);
    dP = y(8:10);
    P = y(11:13);
    I_R_B = Q2R(Q);
    
    % Varying Inertia
    I_b = I_a + m_a * [r_pg(3).^2 0 0; 0 r_pg(3).^2 0; 0 0 0] + I_fm + m_fm * [r_fm(3).^2 0 0; 0 r_fm(3).^2 0; 0 0 0];
    %I_b = diag([0.6 0.6 0.3]); % Body Inertial
    
    %% Newton-Euler equation
    I_thrust = -I_R_B * [0;0;u(1)];
    %B_M = -cross(W, I_b * W) + B_M_f + B_M_d - B_M_g - B_M_a;
    B_M = -cross(W, I_b * W) + u(2:4);

    ddP = [0; 0; g] + I_thrust;
    dW = I_b \ B_M;
    dQ = 0.5 * [-Q(2:4)'; 
                Q(1)*eye(3)+skew(Q(2:4))] * W;
    dydt = [dW; dQ; ddP; dP; zeros([4 1])];
    commands = [0; 0; 0; 0];
    meta = [B_M; zeros([9 1])];
end