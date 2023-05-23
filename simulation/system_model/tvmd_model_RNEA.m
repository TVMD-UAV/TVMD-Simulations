function [dxdt, dzdt, meta, u] = tvmd_model_RNEA(x, z, z_d, env_params, drone_params)
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
    r_pg = drone_params.r_pg;  % Leverage length from c.p. to c.g.
    r_fm = drone_params.r_fm;  % Leverage length from c.fm. to c.g.
    l_pa = drone_params.l_pa;  % Distance between gimbal CoM and propeller
    % end region [Parameters]
    
    pos = drone_params.pos;
    psi = drone_params.psi;
    n = length(psi);
    meta = zeros([6 n]);
    
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

    
    %% Actuator Model
    [A_z, B_z, C_z] = gen_actuator_model(mKp, mKd, pKp);
    % Propeller model
    beta_allo = drone_params.beta_allo;
    P_prop = rho * prop_d^4 * beta_allo;


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
    B_dW_B = -I_bb \ (cross(W, I_bb * W) + C_u(4:6));
    B_dV_B = I_R_B' * [0; 0; -g] + C_u(1:3) / mb;
    V_C  = [W; dP];
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
    % dV_Cn = G.G_C \ ([u(4:6); u(1:3)] + C_F_G + ad(V_C)' * G.G_C * V_C);
    dV_Cn = G.G_C \ ([u(4:6); u(1:3)] + C_F_G - C_F_A + ad(V_C)' * G.G_C * V_C);
    
    % System Dynamics
    ddP = I_R_B * dV_Cn(4:6);
    dW = dV_Cn(1:3);
    dQ = reshape(I_R_B * skew(W), [9 1]);
    meta = C_Fi_A;

    dxdt = [dW; dQ; ddP; dP];
end

function [A_z, B_z, C_z] = gen_actuator_model(mKp, mKd, pKp)
A_z = [   0    1    0    0    0    0;
       -mKp -mKd    0    0    0    0;
          0    0    0    1    0    0;
          0    0 -mKp -mKd    0    0;
          0    0    0    0 -pKp    0;
          0    0    0    0    0 -pKp];
B_z = [   0    0    0    0;
        mKp    0    0    0;
          0    0    0    0;
          0  mKp    0    0;
          0    0  pKp    0;
          0    0    0  pKp];
C_z = [1 0 0 0 0 0;
       0 0 1 0 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];
end

function vec = full_dof_mixing(P, psi, a, b, tf)
    n = length(psi);
    M = get_M(n, psi, P);
    fi = get_f(a, b, tf);
    vec = M * fi;
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