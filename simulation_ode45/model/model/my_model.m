function [dydt, commands, meta] = my_model(params, u, y)
    %% Parameters
    g = params('g'); % gravity

    % Drone
    m = params('m');
    m_a = params('m_a'); % Mass, Kg
    m_fm = params('m_fm');
    r_pg = params('r_pg'); % Leverage length from c.p. to c.g.
    r_fm = params('r_fm'); % Leverage length from c.fm. to c.g.

    I_fm = params('I_fm'); % Body Inertial
    I_a = params('I_a'); % Actuator Inertial

    rho = params('rho'); % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u'); % upper propeller drag coefficient
    CP_l = params('CP_l'); % lower propeller drag coefficient

    mKp = params('mKp'); % Servo motor gain
    mKd = params('mKd'); % Servo motor gain

    %% State variables
    W = y(1:3);
    Q = y(4:7);
    dP = y(8:10);
    P = y(11:13);
    vartheta = y(14:17);
    I_R_B = Q2R(Q);

    [w_m, vartheta_d] = controlAllocation(params, u);
    w_m1 = w_m(1);
    w_m2 = w_m(2);

    % Servo motor dynamics
    d_vartheta = [0 0 1 0;
            0 0 0 1;
            -mKp 0 -mKd 0;
            0 -mKp 0 -mKd] * vartheta + [0 0; 0 0; mKp 0; 0 mKp] * vartheta_d;

    eta = vartheta(1);
    xi = vartheta(2);
    %eta = vartheta_d(1);
    %xi = vartheta_d(2);
    dd_eta = d_vartheta(3);
    dd_xi = d_vartheta(4);

    % Translational thrust
    T_f = rho * w_m1^2 * prop_d^4 * CT_u + rho * w_m2^2 * prop_d^4 * CT_l;
    % Drag torque
    T_d = rho * w_m1^2 * prop_d^5 * CP_u - rho * w_m2^2 * prop_d^5 * CP_l;

    %% System start
    % Gyroscopic moment
    A_w_P = [0; 0; w_m1 - w_m2];
    B_w_A = [vartheta(3); vartheta(4); 0];

    B_R_A = Ry(xi) * Rx(eta);
    B_M_g = B_R_A * I_a * (cross(B_R_A' * B_w_A, A_w_P));
    B_M_a = 0 + B_R_A * I_a * B_R_A' * [dd_eta dd_xi 0]';
    B_I_a = B_R_A * I_a * B_R_A';

    % Thrust torque
    thrust = B_R_A * [0; 0; T_f];
    B_M_f = cross(r_pg, thrust);

    % Drag torque from motor
    B_M_d = B_R_A * [0; 0; T_d];

    % Varying Inertia
    I_b = B_I_a + m_a * [r_pg(3).^2 0 0; 0 r_pg(3).^2 0; 0 0 0] + I_fm + m_fm * [r_fm(3).^2 0 0; 0 r_fm(3).^2 0; 0 0 0];

    %[F_d, M_d] = aerial_drag(params, u, y, false);
    F_d = 0;
    M_d = 0;

    %% Newton-Euler equation
    I_thrust = -I_R_B * thrust / m;
    B_M = -cross(W, I_b * W) + B_M_f + B_M_d - B_M_g - B_M_a + M_d;

    ddP = [0; 0; g] + I_thrust + F_d / m;
    dW = I_b \ B_M;
    dQ = 0.5 * [-Q(2:4)'; Q(1) * eye(3) + skew(Q(2:4))] * W;

    dydt = [dW; dQ; ddP; dP; d_vartheta];
    commands = [w_m1; w_m2; vartheta_d];
    meta = [B_M_f; B_M_d; -B_M_a; -B_M_g];
end

function [w_m, alpha] = controlAllocation(params, u)
    m = params('m'); % Mass, Kg
    rho = params('rho'); % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    r_pg = params('r_pg'); % Leverage length from c.p. to c.g.
    l = r_pg(3);

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u'); % upper propeller drag coefficient
    CP_l = params('CP_l'); % lower propeller drag coefficient

    u_t = m * u(1);
    M_d = sat(u(2:4), [-1; -1; -1], [1; 1; 1]);

    % Control allocator
    Tf = u_t;
    Td = M_d(3);
    A = [-l * Tf Td; -Td -l * Tf];
    alpha = A \ [M_d(1); M_d(2)];
    alpha =- [alpha(1); alpha(2)];

    %% Motor commands
    beta_allo = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
    w_m = sqrt(beta_allo \ [Tf; Td] / (rho * prop_d^4));
end
