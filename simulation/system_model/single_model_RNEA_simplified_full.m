% The simplified model using RNEA method (including translational and rotational parts)
% Using this form, meaningful results can be extracted

function [dxdt, dzdt, meta, T_f, wrench] = single_model_RNEA_simplified(x, z, z_d, env_params, drone_params)
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
    m_p = drone_params.m_p;
    I_b = drone_params.I_b; % Body Inertia
    I_fm = drone_params.I_fm; % Body Inertia
    I_a = drone_params.I_a; % Actuator Inertia
    I_P = drone_params.I_P; % Actuator Inertia
    r_pg = drone_params.r_pg;  % Leverage length from c.p. to c.g.
    r_fm = drone_params.r_fm;  % Leverage length from c.fm. to c.g.
    l_pa = drone_params.l_pa;
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
    reaching_bound_w = ((zi(5:6) <= 0) & (dzdti(5:6) < 0)) | ((zi(5:6) >= sigma_w) & (dzdti(5:6) > 0));
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
    dw_m1 = dzdt(5);
    dw_m2 = dzdt(6);

    theta_P1 = 0;
    theta_P2 = 0;
    
    %% Parameters
    % Transformations
    B_R_A = Ry(eta_y) * Rx(eta_x);
    A_R_P = eye(3);
    B_R_P = B_R_A * A_R_P;

    % Inertia on body frame
    B_J_A = B_R_A * I_a * B_R_A';
    B_J_P = B_R_P * I_P * B_R_P';
    B_J = B_J_A + B_J_P + I_b;

    % Motions (Velocities, Accelerations)
    P_W_P1A = [0; 0; w_m1];
    P_dW_P1A = [0; 0; dw_m1];
    P_W_P2A = [0; 0; -w_m2];
    P_dW_P2A = [0; 0; -dw_m2];
    A_W_AB = [d_eta_x; d_eta_y; 0];
    A_dW_AB = [dd_eta_x; dd_eta_y; 0];
    B_W_B = W;

    % Wrenches from Propellers
    thrust = B_R_A * [0; 0; T_f];
    B_M_f = cross(r_pg, thrust);
    B_T_d = B_R_A * [0; 0; T_d];
    B_V_B = I_R_B' * dP; B_W_B = W;
    B_dV_B = I_R_B' * [0; 0; -g] + thrust / m;
    B_dW_B = -B_J \ (cross(B_W_B, B_J * B_W_B) + B_M_f + B_T_d);

    %% Recursive Newton-Euler Algorithm
    % Forward dynamcis
    A_W_A = B_R_A' * B_W_B + A_W_AB;
    A_dW_A = B_R_A' * B_dW_B + cross(A_W_A, A_W_AB) + A_dW_AB;
    A_V_A = skew(Rx(eta_x)'*[0;0;-r_pg(3)]) * B_R_A' * B_W_B + B_R_A' * B_V_B;
    A_dV_A = skew(Rx(eta_x)'*[0;0;-r_pg(3)]) * B_R_A' * B_dW_B + B_R_A' * B_dV_B + skew(A_V_A) * A_W_AB;

    P_W_P1 = A_R_P' * A_W_A + P_W_P1A;
    P_dW_P1 = A_R_P' * A_dW_A + cross(P_W_P1, P_W_P1A) + P_dW_P1A;
    P_V_P1 = skew([0;0;-l_pa]) * A_R_P' * A_W_A + A_R_P' * A_V_A;
    P_dV_P1 = skew([0;0;-l_pa]) * A_R_P' * A_dW_A + A_R_P' * A_dV_A + skew(P_V_P1) * P_W_P1A;

    P_W_P2 = A_R_P' * A_W_A + P_W_P2A;
    P_dW_P2 = A_R_P' * A_dW_A + cross(P_W_P2, P_W_P2A) + P_dW_P2A;
    P_V_P2 = skew([0;0;l_pa]) * A_R_P' * A_W_A + A_R_P' * A_V_A;
    P_dV_P2 = skew([0;0;l_pa]) * A_R_P' * A_dW_A + A_R_P' * A_dV_A + skew(P_V_P2) * P_W_P2A;


    % Adverse Reactionary Moment
    kA1 = (2*B_J_P + B_J_A) * B_dW_B ...
        + (2*B_J_P + B_J_A) * B_R_A * A_dW_AB ...
        + B_J_P * B_R_P * P_dW_P1A ...
        + B_J_P * B_R_P * P_dW_P2A;

    % Gyroscopic Moment
    B1 = B_W_B + B_R_A * A_W_AB;
    BP1 = B1 + B_R_P * P_W_P1A;
    BP2 = B1 + B_R_P * P_W_P2A;
    kB1 = (2*B_J_P + B_J_A) * cross(B_W_B, B_R_A * A_W_AB) ...
        + B_J_P * cross(B_W_B + B_R_A * A_W_AB, B_R_P * P_W_P1A) ...
        + B_J_P * cross(B_W_B + B_R_A * A_W_AB, B_R_P * P_W_P2A) ...
        + cross(BP1, B_J_P * BP1) ... 
        + cross(BP2, B_J_P * BP2);

    % Delta
    kC1 = cross(B1, B_J_A * B1) ...
        + A_R_P * cross(P_V_P1, m_p * P_V_P1) ...
        + A_R_P * cross(P_V_P2, m_p * P_V_P2) ...
        + cross(A_V_A, m_a * A_V_A);

    % Total Internal Moment from {A} to {B}
    B_T_A = kA1 + kB1 + kC1;

    % Translational
    P_f_P1 = m_p * P_dV_P1 + skew(P_W_P1)*(m_p*P_V_P1);
    P_f_P2 = m_p * P_dV_P2 + skew(P_W_P2)*(m_p*P_V_P2);
    A_f_A = (A_R_P * P_f_P1 + A_R_P * P_f_P2) + m_a * A_dV_A + skew(A_W_A)*(m_a*A_V_A);
    % B_f_A = B_R_A * A_f_A;
    % fprintf("%.8f, %.8f, %.8f \n", B_f_A(1), B_f_A(2), B_f_A(3))
    disp(A_dV_A)

    %% Dynamics
    % Translational
    I_thrust = I_R_B * thrust;
    % thrust = [0;0; m*g];
    % B_dv_B = (I_R_B' * [0; 0; -g] + thrust / m);
    % B_dv_B = (I_R_B' * [0; 0; -g] + thrust / m) - B_R_A * A_f_A - m * cross(W, dP);
    B_dv_B = (I_R_B' * [0; 0; -g] + thrust / m) - B_R_A * A_f_A;
    ddP = I_R_B * B_dv_B;
    
    % Rotational
    B_M = -cross(W, I_b * W) - B_T_A + B_T_d + B_M_f;
    dW = B_J \ B_M;
    dQ = reshape(I_R_B * skew(W), [9 1]);

    wrench = [I_thrust; B_M];
    meta = [B_M_f B_T_d -kB1 -kA1 -kC1];

    dxdt = [dW; dQ; ddP; dP];
end
