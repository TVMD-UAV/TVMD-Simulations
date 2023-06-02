% The original RNEA method (including translational and rotational parts)

function [dxdt, dzdt, meta, T_f, wrench] = single_model_RNEA_full(x, z, z_d, env_params, drone_params)
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
    dw_m1 = dzdt(5);
    dw_m2 = dzdt(6);

    theta_P1 = 0;
    theta_P2 = 0;

    %% Parameters
    % Transformations
    B_R_A = Ry(eta_y) * Rx(eta_x);

    % Motions (Velocities, Accelerations)
    % l_pa = 0;
    % r_pg(3) = 0;
    A_T_C = [Rx(eta_x)'*Ry(eta_y)' Rx(eta_x)'*[0;0;-r_pg(3)];
             0 0 0 1];
    dA_A =  [d_eta_x; d_eta_y; 0; 0; 0; 0];
    ddA_A = [dd_eta_x; dd_eta_y; 0; 0; 0; 0];

    P1_T_A = [Rz(theta_P1)' [0;0;-l_pa];
            0 0 0 1];
    P2_T_A = [Rz(theta_P2)' [0;0;l_pa];
            0 0 0 1];
    dA_P1 = [0; 0; w_m1; 0; 0; 0];
    ddA_P1 = [0; 0; dw_m1; 0; 0; 0];
    dA_P2 = [0; 0; -w_m2; 0; 0; 0];
    ddA_P2 = [0; 0; -dw_m2; 0; 0; 0];

    G_A = [I_a zeros([3 3]);
        zeros([3 3]) m_a * eye(3)];
    G_P = [I_P zeros([3 3]);
        zeros([3 3]) m_p * eye(3)];
    G_B = [I_fm zeros([3 3]);
        zeros([3 3]) m_fm * eye(3)];

    % Wrenches from Propellers
    thrust = B_R_A * [0; 0; T_f];
    B_M_f = cross(r_pg, thrust);
    P_T_d = [0; 0; T_d];
    I_thrust = I_R_B * thrust;
    C_F_e = [B_M_f + B_R_A * P_T_d; m * [0; 0; -g] + I_thrust];

    B_V_B = I_R_B' * dP; B_W_B = W;
    B_dW_B = -I_b \ (cross(B_W_B, I_b * B_W_B) + B_M_f + B_R_A * P_T_d);
    B_dV_B = I_R_B' * [0; 0; -g] + thrust / m;
    V_C  = [W; B_V_B];
    dV_C = [B_dW_B; B_dV_B];

    %% Recursive Newton-Euler Algorithm
    % Forward Iteration
    V_A = Ad(A_T_C) * V_C + dA_A;
    dV_A = Ad(A_T_C) * dV_C + ad(V_A) * dA_A + ddA_A;

    V_P1 = Ad(P1_T_A) * V_A + dA_P1;
    dV_P1 = Ad(P1_T_A) * dV_A + ad(V_P1) * dA_P1 + ddA_P1;

    V_P2 = Ad(P2_T_A) * V_A + dA_P2;
    dV_P2 = Ad(P2_T_A) * dV_A + ad(V_P2) * dA_P2 + ddA_P2;

    % Backward Iteration
    P_F_P1 = G_P * dV_P1 - ad(V_P1)' * (G_P * V_P1);
    P_F_P2 = G_P * dV_P2 - ad(V_P2)' * (G_P * V_P2);
    A_F_A = Ad(P1_T_A)' * P_F_P1 + Ad(P2_T_A)' * P_F_P2 + G_A * dV_A - ad(V_A)' * (G_A * V_A);
    

    %% Dynamics
    dV_Cn = G_B \ (C_F_e - Ad(A_T_C)' * A_F_A + ad(V_C)' * G_B * V_C);
    
    dQ = reshape(I_R_B * skew(W), [9 1]);
    dW = dV_Cn(1:3);
    ddP = I_R_B * dV_Cn(4:6);

    C_F_A = - Ad(A_T_C)' * A_F_A;
    C_F_A2 = ad(V_C)' * G_B * V_C;
    wrench = [I_thrust; B_M_f + B_R_A * P_T_d];
    meta = [B_M_f B_R_A*P_T_d C_F_A(1:3) C_F_A2(1:3) dW];

    dxdt = [dW; dQ; ddP; dP];
end

function AdT = Ad(T)
    AdT = [T(1:3, 1:3) zeros([3 3]);
           skew(T(1:3,4))*T(1:3, 1:3) T(1:3, 1:3)];
end
 
function adV = ad(V)
    adV = [skew(V(1:3)) zeros([3 3]);
           skew(V(4:6)) skew(V(1:3))];
end 
 