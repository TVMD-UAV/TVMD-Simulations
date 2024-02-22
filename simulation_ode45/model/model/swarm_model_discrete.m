% u: the [F a b] command for individual agent
% y: state
function [dydt, commands, meta, vec] = swarm_model_discrete(params, t, F_d, a_d, b_d, y)
    %% Parameters
    g = params('g'); % gravity
    rho = params('rho'); % kg/m3
    prop_d = params('prop_d'); % 8 inch = 20.3 cm

    CT_u = params('CT_u'); % upper propeller thrust coefficient
    CT_l = params('CT_l'); % lower propeller thrust coefficient
    CP_u = params('CP_u'); % upper propeller drag coefficient
    CP_l = params('CP_l'); % lower propeller drag coefficient

    mKp = params('mKp'); % Servo motor gain
    mKd = params('mKd'); % Servo motor gain
    pKp = params('pKp'); % BLDC motor gain

    % Bounds
    sigma_a = params('sigma_a');
    sigma_b = params('sigma_b');
    r_sigma_a = params('r_sigma_a');
    r_sigma_b = params('r_sigma_b');
    f_max = params('f_max');
    r_f = params('r_f');
    prop_max = params('prop_max');

    % Drone
    m = params('m');
    I_b = params('I_b'); % Body Inertial

    pos = params('pos');
    psi = params('psi');
    n = length(psi);

    %% State variables
    W = y(1:3);
    Q = reshape(y(4:12), [3 3]); % 3x3
    dP = y(13:15);
    P = y(16:18);

    % agent motors
    vtheta = reshape(y(19:end), [6 n]);

    % Servo model
    A_motor = [0 0 1 0;
            0 0 0 1;
            -mKp 0 -mKd 0;
            0 -mKp 0 -mKd];
    B_motor = [0 0; 0 0; mKp 0; 0 mKp];
    C_motor = [1 0 0 0; 0 1 0 0];

    % Propeller model
    beta_allo = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
    P_prop = rho * prop_d^4 * beta_allo;

    dm = zeros([6 * n 1]);
    F = zeros([n 1]);
    a = zeros([n 1]);
    b = zeros([n 1]);

    for i = 1:n
        wm = vtheta(1:2, i);
        ws = vtheta(3:6, i);
        wsi_d = [a_d(i); b_d(i)];

        % Motor commands
        wm_d = sqrt(beta_allo \ [F_d(i); 0] / (rho * prop_d^4));

        % Motor models
        d_wmi = -pKp * wm + pKp * wm_d;
        d_wsi = A_motor * ws + B_motor * wsi_d;

        wm = min(prop_max, max(0, wm));

        % Rate constraint
        r_prop_max = sqrt(inv(beta_allo) / (rho * prop_d^4)) * [r_f / (2 * sqrt(abs(F_d(i)))); 0];

        d_wmi = min(r_prop_max, max(-r_prop_max, d_wmi));
        reaching_bound_w = ((wm <= 0) & (d_wmi <= 0)) | ((wm >= prop_max) & (d_wmi >= 0));
        d_wmi = (~reaching_bound_w) .* d_wmi;

        d_wsi(1:2) = min([r_sigma_a; r_sigma_b], max(- [r_sigma_a; r_sigma_b], d_wsi(1:2)));
        reaching_bound_a = ((ws(1) < -sigma_a) && (d_wsi(1) < 0)) || ((ws(1) > sigma_a) && (d_wsi(1) > 0));
        reaching_bound_b = ((ws(2) < -sigma_b) && (d_wsi(2) < 0)) || ((ws(2) > sigma_b) && (d_wsi(2) > 0));
        d_wsi(1) = (~reaching_bound_a) * d_wsi(1);
        d_wsi(2) = (~reaching_bound_b) * d_wsi(2);

        dm(6 * (i - 1) + 1:6 * i) = [d_wmi; d_wsi];

        % parsing
        TfTd = P_prop * (wm.^2);
        % F(i) = min(f_max, max(0, TfTd(1)));
        F(i) = TfTd(1);
        a(i) = min(sigma_a, max(-sigma_a, ws(1))); % eta
        b(i) = min(sigma_b, max(-sigma_b, ws(2))); % xi
    end

    % get total control input
    vec = full_dof_mixing(pos, psi, a, b, F);

    if t > 20 && t < 3
        v_w = [3; 3; 0];
    else
        v_w = [0; 0; 0];
    end
    params('v_w') = v_w;
    [F_dist, M_dist] = aerial_drag(params, norm(vec(1:3)), y, true);
    % F_dist = 0; M_dist = 0;

    %% Newton-Euler equation
    I_thrust = Q * vec(1:3);
    B_M = -cross(W, I_b * W) + vec(4:6);
    B_M_f = vec(4:6);
    B_M_d = [0; 0; 0];

    ddP = ([0; 0; -m * g] + I_thrust + F_dist) / m;
    dW = I_b \ (B_M + M_dist);
    dQ = reshape(Q * skew(W), [9 1]);

    dydt = [dW; dQ; ddP; dP; dm];
    commands = vec;
    %meta = [B_M_f; B_M_d;zeros(6, 1)];%zeros(12, 1); %[B_M_f; B_M_d; -B_M_a; -B_M_g];
    meta = [B_M_f; B_M_d; B_M; zeros(3, 1)]; %zeros(12, 1); %[B_M_f; B_M_d; -B_M_a; -B_M_g];
end

function sysd = calc_actuator_discrete_model(params, dt)
    mKp = params('mKp'); % Servo motor gain
    mKd = params('mKd'); % Servo motor gain

    A = [0 0 1 0;
    0 0 0 1;
    -mKp 0 -mKd 0;
    0 -mKp 0 -mKd];
    B = [0 0; 0 0; mKp 0; 0 mKp];
    C = [1 0 0 0; 0 1 0 0];
    motor_sys = sys(A, B, C, 0);
    sysd = c2d(motor_sys, dt);
end