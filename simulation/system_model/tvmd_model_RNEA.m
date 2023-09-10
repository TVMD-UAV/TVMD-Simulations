function [dxdt, dzdt, meta, u] = tvmd_model_RNEA(t, x, z, z_d, u_d_bypass, env_params, drone_params, ctrl_params)
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

    if drone_params.model_uncertain
        m = drone_params.m_uncertain;        % Single Body Mass
        mb = drone_params.mb_uncertain;      % Team Body Mass
        m_p = drone_params.m_p_uncertain;    % Single agent actuator mass, Kg
        m_a = drone_params.m_a_uncertain;    % Single agent actuator mass, Kg
        m_fm = drone_params.m_fm_uncertain;  % Single agent frame mass, Kg

        I_b = drone_params.I_b_uncertain;    % Single Body Inertia
        I_bb = drone_params.I_bb_uncertain;  % Team Body Inertia
        I_a = drone_params.I_a_uncertain;    % Actuator Inertia
        I_P = drone_params.I_P_uncertain;    % Actuator Inertia
        I_fm = drone_params.I_fm_uncertain;   % Frame Inertia
    else
        m = drone_params.m;        % Single Body Mass
        mb = drone_params.mb;      % Team Body Mass
        m_p = drone_params.m_p;    % Single agent actuator mass, Kg
        m_a = drone_params.m_a;    % Single agent actuator mass, Kg
        m_fm = drone_params.m_fm;  % Single agent frame mass, Kg

        I_b = drone_params.I_b;    % Single Body Inertia
        I_bb = drone_params.I_bb;  % Team Body Inertia
        I_a = drone_params.I_a;    % Actuator Inertia
        I_P = drone_params.I_P;    % Actuator Inertia
        I_fm = drone_params.I_fm;   % Frame Inertia
    end
    r_pg = drone_params.r_pg;  % Leverage length from c.p. to c.g.
    r_fm = drone_params.r_fm;  % Leverage length from c.fm. to c.g.
    l_pa = drone_params.l_pa;  % Distance between gimbal CoM and propeller
    % end region [Parameters]
    
    pos = drone_params.pos;
    psi = drone_params.psi;
    n = length(psi);
    meta = zeros([6 5]);
    
    % State extraction
    W = x(1:3);
    I_R_B = reshape(x(4:12), [3 3]); % 3x3
    dP = x(13:15);
    P = x(16:18);
    
    dzdt = zeros([6 * n 1]);
    Tf = zeros([n 1]);
    eta_x = zeros([n 1]);
    eta_y = zeros([n 1]);
    C_Fi_A = zeros([6 n]);

    if ctrl_params.ud_pypass
        u = u_d_bypass;
        ddP = [0;0;-g] + (I_R_B * u(1:3)) / mb;
        dW = I_bb \ (sat(u(4:6), -[100;100;100], [100;100;100]) - cross(W, I_bb * W));
        dQ = reshape(I_R_B * skew(W), [9 1]);
        meta(1:3, 1:2) = [u(4:6) -cross(W, I_bb * W)];

        dxdt = [dW; dQ; ddP; dP];
        return;
    end

    
    %% Actuator Model
    [A_z, B_z, C_z] = gen_actuator_model(mKp, mKd, pKp);
    % Propeller model
    beta_allo = drone_params.beta_allo;
    P_prop = rho * prop_d^4 * beta_allo;

    % if ctrl_params.zd_pypass
    %     z_d = z_d_bypass;
    %     for i = 1:n
    %         z(6*(i-1)+1 : 6*i) = C_z' * z_d_bypass(4*(i-1)+1 : 4*i);
    %     end
    % end

    % Parameters
    G.G_A = [I_a zeros([3 3]);
    zeros([3 3]) m_a * eye(3)];
    G.G_P = [I_P zeros([3 3]);
        zeros([3 3]) m_p * eye(3)];
    G.G_B = [I_fm zeros([3 3]);
        zeros([3 3]) m_fm * eye(3)];
    G_C = zeros([3 3]);
    for i = 1:n
        G_C = G_C - m_fm * skew(pos(:, i)) * skew(pos(:, i));
    end
    G.G_C = [G_C zeros([3 3]);
            zeros([3 3]) n * m_fm * eye(3)];


    % % Motor Failure
    % N_esp = ones([n 1]);
    % if drone_params.agent_disable
    %     if any((t > drone_params.agent_disable_time(:, 1)) & (t <= drone_params.agent_disable_time(:, 2)))
    %         for m=1:length(drone_params.agent_disable_id)
    %             N_esp(drone_params.agent_disable_id(m)) = 0;
    %         end
    %     end
    % end

    % Acceleration Estimation
    for i = 1:n
        zi = z(6 * (i - 1) + 1 : 6 * i);
        zio = C_z * zi;
        TfTdi = P_prop * zio(3:4, 1).^2;                         % [u_f; u_tau]
        Tf(i) = TfTdi(1);
        eta_x(i) = zio(1);
        eta_y(i) = zio(2);
    end

    C_u = full_dof_mixing(pos, psi, eta_x, eta_y, Tf);
    B_dW_B = I_bb \ (C_u(4:6) - cross(W, I_bb * W));
    B_dV_B = I_R_B' * [0; 0; -g] + C_u(1:3) / mb;
    V_C  = [W; I_R_B' * dP];
    dV_C = [B_dW_B; B_dV_B];
    
    for i = 1:n
        % Actuator Model
        zi = z(6 * (i - 1) + 1 : 6 * i);
        z_di = z_d(4*(i-1)+1 : 4*i);

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

        dzdt(6 * (i - 1) + 1 : 6 * i) = dzdti;
    
        zio = C_z * zi;
        TfTdi = P_prop * zio(3:4, 1).^2;                         % [u_f; u_tau]
        Tf(i) = TfTdi(1);
        eta_x(i) = zio(1);
        eta_y(i) = zio(2);
        % end region [Actuator Dynamics]

        %% Parameters
        theta_P1 = 0;
        theta_P2 = 0;

        Trans.A_T_C = [Rx(eta_x(i))'*Ry(eta_y(i))' Rx(eta_x(i))'*[0;0;-r_pg(3)];
                       0 0 0 1];
        Trans.P1_T_A = [Rz(theta_P1)' [0;0;-l_pa];
                        0 0 0 1];
        Trans.P2_T_A = [Rz(theta_P2)' [0;0;l_pa];
                        0 0 0 1];
        dA.dA_A =  [dzdti(1); dzdti(3); 0; 0; 0; 0];
        ddA.ddA_A = [dzdti(2); dzdti(4); 0; 0; 0; 0];
        dA.dA_P1 = [0; 0; zio(3); 0; 0; 0];
        ddA.ddA_P1 = [0; 0; dzdti(5); 0; 0; 0];
        dA.dA_P2 = [0; 0; -zio(4); 0; 0; 0];
        ddA.ddA_P2 = [0; 0; -dzdti(6); 0; 0; 0];

        Bi_T_C = [Rz(psi(i))' -pos(:, i); 
                  zeros([1 3]) 1];
        V_B = Ad(Bi_T_C) * V_C;
        dV_B = Ad(Bi_T_C) * dV_C;

        A_Fi_A = RNEA_single(Trans, V_B, dV_B, dA, ddA, G);
        C_Fi_A(:, i) = Ad(Trans.A_T_C * Bi_T_C)' * A_Fi_A;
    end
    
    
    u = full_dof_mixing(pos, psi, eta_x, eta_y, Tf);
    C_F_A = sum(C_Fi_A, 2);
    C_F_G= [zeros([3 1]); I_R_B' * [0;0;-mb*g]];

    C_F_E = zeros([6 1]);
    if env_params.ext_wind
        C_F_E = aerial_drag(env_params, drone_params, u, I_R_B, dP);
    end 

    if env_params.ext_dist
        C_f_L = -I_R_B' * env_params.ext_load_weight * env_params.g * [0;0;1];
        C_M_L = cross(env_params.ext_load_pos, C_f_L);
        C_F_E = C_F_E + [C_M_L; C_f_L] + env_params.ext_dist_wrench;
        % todo: the induced motion due to the motion of the payload
    end

    C_F_T = [u(4:6); u(1:3)];
    C_F_Delta = ad(V_C)' * G.G_C * V_C;
    if ctrl_params.internal_moment_bypass
        dV_Cn = G.G_C \ (C_F_T + C_F_G + C_F_Delta + C_F_E);
    else
        dV_Cn = G.G_C \ (C_F_T + C_F_G - C_F_A + C_F_Delta + C_F_E);
    end
    
    % System Dynamics
    ddP = I_R_B * dV_Cn(4:6);
    dW = dV_Cn(1:3);
    dQ = reshape(I_R_B * skew(W), [9 1]);
    meta = [C_F_T  C_F_G  (-C_F_A)  C_F_E  C_F_Delta];

    dxdt = [dW; dQ; ddP; dP];
end

function A_F_A = RNEA_single(Trans, V_C, dV_C, dA, ddA, G)
    A_T_C=Trans.A_T_C; 
    P1_T_A=Trans.P1_T_A; 
    P2_T_A=Trans.P2_T_A;
    dA_A=dA.dA_A; 
    dA_P1=dA.dA_P1; 
    dA_P2=dA.dA_P2; 
    ddA_A=ddA.ddA_A; 
    ddA_P1=ddA.ddA_P1; 
    ddA_P2=ddA.ddA_P2;
    G_P=G.G_P; 
    G_A=G.G_A;
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
end


function AdT = Ad(T)
    AdT = [T(1:3, 1:3) zeros([3 3]);
           skew(T(1:3,4))*T(1:3, 1:3) T(1:3, 1:3)];
end
 
function adV = ad(V)
    adV = [skew(V(1:3)) zeros([3 3]);
           skew(V(4:6)) skew(V(1:3))];
end 

function C_F_E = aerial_drag(env_params, drone_params, u, R, v)
    % Drone parameters
    rho = env_params.rho;
    v_w = env_params.ext_v_w;
    C_d = drone_params.C_d;
    I_xy = drone_params.I_xy;
    A_cs = drone_params.A_cs;
    esp_M = drone_params.esp_M;

    %% Aerial Dynamics
    F_drag = norm(v_w - v) * R * C_d * R' * (v_w - v);
    F_ram = sqrt(u(3) * rho * A_cs / 2) * R * I_xy * R' * (v_w - v);
    F_d = F_drag + F_ram;
    M_d = esp_M * skew([0; 0; 1]) * R' * F_d;
    C_F_E = [M_d; R' * F_d];
end
