function [dydt, commands, meta] = bi_rotor_model(u, y)
    [key, params] = get_birotor_params();
    g = params('g'); % g

    I_b = params('I_b');
    I_a = params('I_a');
    I_r = params('I_r');
    m = params('m');
    l = params('l');
    h = params('h');

    mKp = params('mKp');
    mKd = params('mKd');

    %% State variables
    W = y(1:3);
    Q = reshape(y(4:12), [3 3]); % 3x3
    dP = y(13:15);
    P = y(16:18);
    vartheta = y(19:22);

    %% Control allocation
    [w_m, vartheta_d, P1, P2, Td1, Td2] = controlAllocation(params, u);

    % Servo motor dynamics
    d_vartheta = [0 0 1 0;
            0 0 0 1;
            -mKp 0 -mKd 0;
            0 -mKp 0 -mKd] * vartheta + [0 0; 0 0; mKp 0; 0 mKp] * vartheta_d;
    alpha1 = vartheta(1);
    alpha2 = vartheta(2);
    %alpha1 = vartheta_d(1);
    %alpha2 = vartheta_d(2);
    d_alpha1 = vartheta(3);
    d_alpha2 = vartheta(4);
    dd_alpha1 = d_vartheta(3);
    dd_alpha2 = d_vartheta(4);

    %% Forces and moments
    thrust = Ry(alpha1) * [0; 0; P1] + Ry(alpha2) * [0; 0; P2];
    w_m1 = w_m(1);
    w_m2 = w_m(2);
    M_gyro = I_r * [-w_m1 * d_alpha1 * cos(alpha1) + w_m2 * d_alpha2 * cos(alpha2); 0;
                w_m1 * d_alpha1 * sin(alpha1) - w_m2 * d_alpha2 * sin(alpha2)];
    M_drag = [- (Td1 * sin(alpha1) - Td2 * sin(alpha2)); 0;
        - (Td1 * cos(alpha1) - Td2 * cos(alpha2))];
    M_thrust = [l * (P1 * cos(alpha1) - P2 * cos(alpha2));
            h * (P1 * sin(alpha1) + P2 * sin(alpha2));
            -l * (P1 * sin(alpha1) - P2 * sin(alpha2))];
    M_react =- I_a * (dd_alpha1 + dd_alpha2) * [0; 1; 0];

    %% Newton-Euler equation
    B_M = -cross(W, I_b * W) + M_thrust + M_drag - M_react - M_gyro;
    %B_M = -cross(W, I_b * W) + M_thrust - M_react - M_gyro;

    dW = I_b \ B_M;
    dQ = reshape(Q * skew(W), [9 1]);

    I_thrust = Q * thrust / m;

    ddP = [0; 0; -g] + I_thrust;
    dydt = [dW; dQ; ddP; dP; d_vartheta];
    commands = [w_m; vartheta_d];
    meta = [M_thrust; M_drag; -M_react; -M_gyro];
end

function [w_m, alpha, P1, P2, Td1, Td2] = controlAllocation(params, u)
    m = params('m');
    l = params('l');
    h = params('h');
    rho = params('rho'); % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u'); % upper propeller drag coefficient
    CP_l = params('CP_l'); % lower propeller drag coefficient

    u_t = m * u(1);
    M = u(2:4);

    %% Control allocation
    %alpha1 = atan2(l*(l*u_t+M(1)), M(2)/h-M(3)/l);
    %alpha2 = atan2(l*(l*u_t-M(1)), M(2)/h+M(3)/l);

    %P1 = (l*u_t+M(1)) / (2*l*cos(alpha1))
    %P2 = (l*u_t-M(1)) / (2*l*cos(alpha2))

    % alpha1 = atan2(l*h*u_t+h*M(2), l*M(1)+h*M(3));
    % alpha2 = atan2(l*h*u_t-h*M(2), l*M(1)-h*M(3));
    % P1 = 0.5 * sqrt((M(1)/h+M(3)/l)^2 + (u_t+M(2)/l)^2);
    % P2 = 0.5 * sqrt((M(1)/h-M(3)/l)^2 + (u_t-M(2)/l)^2);

    AA = [0 1 0 1; 0 l 0 -l; h 0 h 0; -l 0 l 0];
    k = AA \ [u_t; M];
    alpha1 = atan2(k(1), k(2));
    alpha2 = atan2(k(3), k(4));
    P1 = sqrt(k(1)^2 + k(2)^2);
    P2 = sqrt(k(3)^2 + k(4)^2);

    % Propeller
    Td1 = P1 * prop_d * CT_u / CP_u;
    Td2 = P2 * prop_d * CT_l / CP_l;
    w_m1 = sqrt(P1 / (rho * prop_d^4 * CT_u));
    w_m2 = sqrt(P2 / (rho * prop_d^4 * CT_l));
    w_m = [w_m1; w_m2];
    alpha = [alpha1; alpha2];
end
