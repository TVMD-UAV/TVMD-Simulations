function [dxdt, dzdt, meta, T_f, wrench] = single_model(x, z, z_d, env_params, drone_params)
    % region [Parameters]
    g = env_params.g; % gravity
    rho = env_params.rho; % kg/m3
    prop_d = env_params.prop_d; % 8 inch = 20.3 cm
    
    CT_u = env_params.CT_u; % upper propeller thrust coefficient
    CT_l = env_params.CT_l; % lower propeller thrust coefficient
    CP_u = env_params.CP_u; % upper propeller drag coefficient
    CP_l = env_params.CP_l; % lower propeller drag coefficient
    
    mKp = drone_params.mKp; % Servo motor gain
    mKd = drone_params.mKd; % Servo motor gain
    pKp = drone_params.pKp; % BLDC motor gain
    
    % Bounds
    sigma_x = drone_params.sigma_a;
    sigma_y = drone_params.sigma_b;
    r_sigma_x = drone_params.r_sigma_a;
    r_sigma_y = drone_params.r_sigma_b;
    
    % TODO:
    sigma_w0 = drone_params.sigma_w0;
    sigma_w = drone_params.prop_max;
    r_sigma_w = drone_params.r_sigma_w;
    r_f = drone_params.r_f;
    
    % Drone
    m = drone_params.m;     % Body Mass
    m_a = drone_params.m_a;    % Single agent actuator mass, Kg
    m_fm = drone_params.m_fm;  % Single agent frame mass, Kg
    I_b = drone_params.I_b; % Body Inertia
    I_fm = drone_params.I_fm; % Body Inertia
    I_a = drone_params.I_a; % Actuator Inertia
    I_P = drone_params.I_P; % Actuator Inertia
    r_pg = drone_params.r_pg;  % Leverage length from c.p. to c.g.
    r_fm = drone_params.r_fm;  % Leverage length from c.fm. to c.g.
    % end region [Parameters]

    % State extraction
    W = x(1:3);
    I_R_B = reshape(x(4:12), [3 3]); % 3x3
    dP = x(13:15);
    P = x(16:18);

    %% Actuator Model
    [A_z, B_z, C_z] = gen_actuator_model(mKp, mKd, pKp);
    % Propeller model
    % beta_allo = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
    beta_allo = drone_params.beta_allo;
    P_prop = rho * prop_d^4 * beta_allo;

    zi = z;
    z_di = z_d;

    % region [Actuator Dynamics]
    % Saturation
    z_low = [-sigma_x; -r_sigma_x; -sigma_y; -r_sigma_y; sigma_w0; sigma_w0];
    z_upp = [ sigma_x;  r_sigma_x;  sigma_y;  r_sigma_y;  sigma_w(1);  sigma_w(2)];
    zi = min(z_upp, max(z_low, zi));
    dzdti = A_z * zi + B_z * z_di;

    %r_prop_max = sqrt(inv(beta_allo) / (rho * prop_d^4)) * [r_f / (2 * sqrt(abs(Tf_d(i)))); 0];
    r_prop_max = r_sigma_w;
    dz_low = [-r_sigma_x; -inf; -r_sigma_x; -inf; -r_prop_max; -r_prop_max];
    dz_upp = [ r_sigma_x;  inf;  r_sigma_x;  inf;  r_prop_max;  r_prop_max];
    dzdti = min(dz_upp, max(dz_low, dzdti));
    reaching_bound_x = ((zi(1) < -sigma_x) && (dzdti(1) < 0)) || ((zi(1) > sigma_x) && (dzdti(1) > 0));
    reaching_bound_y = ((zi(3) < -sigma_y) && (dzdti(3) < 0)) || ((zi(3) > sigma_y) && (dzdti(3) > 0));
    reaching_bound_w = ((zi(5:6) <= 0) & (dzdti(5:6) <= 0)) | ((zi(5:6) >= sigma_w) & (dzdti(5:6) >= 0));
    dzdti(1) = (~reaching_bound_x) * dzdti(1);
    dzdti(3) = (~reaching_bound_y) * dzdti(3);
    dzdti(5:6) = (~reaching_bound_w) .* dzdti(5:6);

    %dzdt(6 * (i - 1) + 1 : 6 * i) = A_z * zi + B_z * z_d;   % unbounded
    dzdt = dzdti;
    zio = C_z * zi;
    TfTdi = P_prop * zio(3:4, 1).^2;
    T_f = TfTdi(1);
    T_d = TfTdi(2);
    eta_x = zio(1);
    d_eta_x = dzdt(1);
    dd_eta_x = dzdt(2);
    eta_y = zio(2);
    d_eta_y = dzdt(3);
    dd_eta_y = dzdt(4);
    w_m1 = zio(3);
    w_m2 = zio(4);

    % Gyroscopic moment
    A_w_P = [0; 0; w_m1 - w_m2];
    A_w_A = [d_eta_x; d_eta_y; 0];
    B_R_A = Ry(eta_y) * Rx(eta_x);

%     B_M_g = B_R_A * I_a * (cross(B_R_A' * B_w_A, A_w_P));
%     B_M_a = 0 + B_R_A * I_a * B_R_A' * [dd_eta_x dd_eta_y 0]';
    B_I_a = B_R_A * I_a * B_R_A';
    B_I_P = B_R_A * I_P * B_R_A';
    B_M_g = - B_I_P * B_R_A * cross(A_w_A, A_w_P);
    B_M_a = - (B_I_a) * B_R_A * [dd_eta_x; dd_eta_y; 0];
    B_M_delta = - cross(B_R_A * A_w_A, B_I_a * B_R_A * A_w_A);
    % Varying Inertia
    % I_b = B_I_a + m_a * [r_pg(3).^2 0 0; 0 r_pg(3).^2 0; 0 0 0] + I_fm + m_fm * [r_fm(3).^2 0 0; 0 r_fm(3).^2 0; 0 0 0];

    % Thrust torque
    thrust = B_R_A * [0; 0; T_f];
    B_M_f = cross(r_pg, thrust);

    % Drag torque from motor
    B_M_d = B_R_A * [0; 0; T_d];
    I_thrust = I_R_B * thrust;
    B_M = -cross(W, I_b * W) + B_M_f + B_M_d + B_M_g + B_M_a;
%     B_M = -cross(W, I_b * W) + B_M_f + B_M_d;
    meta = [B_M_f B_M_d B_M_g B_M_a B_M_delta];

    ddP = ([0; 0; -g] + I_thrust / m);

    wrench = [I_thrust; B_M];
    
    % Rotational
    dW = I_b \ B_M;
    dQ = reshape(I_R_B * skew(W), [9 1]);

    dxdt = [dW; dQ; ddP; dP];
end
