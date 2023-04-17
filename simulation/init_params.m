% [key, params] = get_swarm_params("model_A9_con");
conf_name = "model_A9_inc";
% function [key, params] = get_swarm_params(conf_name)
    % Environment
env_params.g = 9.818; % gravity
env_params.rho = 1.225; % kg/m3
env_params.prop_d = 0.0254 * 9; % 8 inch = 20.3 cm

env_params.CT_u = 0.020231; % upper propeller thrust coefficient
env_params.CT_l = 0.020231; % lower propeller thrust coefficient
env_params.CP_u = 0.0188; % upper propeller drag coefficient
env_params.CP_l = 0.0188; % lower propeller drag coefficient

% Drone
m_a = 0.126; % Mass, Kg
m_fm = 0.416;
m = m_a + m_fm;
drone_params.r_pg = [0; 0; 0.03]; % Leverage length from c.p. to c.g.
drone_params.r_fm = [0; 0; -0.02]; % Leverage length from c.fm. to c.g.

I_fm = [0.0031 0 0; 0 0.0064 0; 0 0 0.0040]; % Body Inertial
I_a = [0.0005 0 0; 0 0.0002 0; 0 0 0.0003]; % Actuator Inertial

drone_params.mKp = 200;
drone_params.mKd = 20;
drone_params.pKp = 1000;

dt = 0.01;

    % Kp = 0.05;
    % Kv = 0.2;
    % Gamma_v = diag([0.2 0.2 0.8]);
    % gamma_q = 1;
    % gamma_theta1 = 0.2;
    % gamma_theta2 = 1;
    % gamma_theta3 = 1;
    % k_theta = 1;
    % Kq = 2 * diag([0.5 1 1]);
    % Kw = 2 * diag([0.5 1 1]);

    % delta_a = 5;
    % delta_b = 3;

    % v_w = [2; 2; 0];
    % C_d = diag([1 1 0.5]);
    % I_xy = diag([1 1 0]);
    % A_cs = 0.114 * 10;
    % esp_M = 0.1;

GRID_SIZE = 0.5 * sqrt(2) / 2;
if conf_name == "model_A3_con"
    [pos, psi] = model_A3_con(GRID_SIZE);    
elseif conf_name == "model_A4_con"
    [pos, psi] = model_A4_con(GRID_SIZE);
elseif conf_name == "model_A5_con"
    [pos, psi] = model_A5_con(GRID_SIZE);
elseif conf_name == "model_A6_con"
    [pos, psi] = model_A6_con(GRID_SIZE);
elseif conf_name == "model_A7_con"
    [pos, psi] = model_A7_con(GRID_SIZE);
elseif conf_name == "model_A8_con"
    [pos, psi] = model_A8_con(GRID_SIZE);
elseif conf_name == "model_A9_con"
    [pos, psi] = model_A9_con(GRID_SIZE);
elseif conf_name == "model_A10_con"
    [pos, psi] = model_A10_con(GRID_SIZE);
elseif conf_name == "model_A3_inc"
    [pos, psi] = model_A3_inc(GRID_SIZE);    
elseif conf_name == "model_A4_inc"
    [pos, psi] = model_A4_inc(GRID_SIZE);
elseif conf_name == "model_A5_inc"
    [pos, psi] = model_A5_inc(GRID_SIZE);
elseif conf_name == "model_A6_inc"
    [pos, psi] = model_A6_inc(GRID_SIZE);
elseif conf_name == "model_A7_inc"
    [pos, psi] = model_A7_inc(GRID_SIZE);
elseif conf_name == "model_A8_inc"
    [pos, psi] = model_A8_inc(GRID_SIZE);
elseif conf_name == "model_A9_inc"
    [pos, psi] = model_A9_inc(GRID_SIZE);
elseif conf_name == "model_A10_inc"
    [pos, psi] = model_A10_inc(GRID_SIZE);
elseif conf_name == "model_T10_con"
    [pos, psi] = model_T10_con(GRID_SIZE);
elseif conf_name == "model_T10_inc"
    [pos, psi] = model_T10_inc(GRID_SIZE);
end

drone_params.pos = pos;
drone_params.psi = psi;

I_b = zeros(3);

for i = 1:length(pos)
    %x2 = pos(:, i).^2;
    %I_b = I_b + I_fm + m * diag([x2(2) + x2(3); x2(1) + x2(3); x2(2) + x2(1)]);
    I_b = I_b + I_fm + m * (pos(:, i)' * pos(:, i) * eye(3) - pos(:, i) * pos(:, i)');
end
drone_params.I_b = I_b;
drone_params.m = m * length(pos);

%% Constraints
drone_params.sigma_a = pi / 6;
drone_params.sigma_b = pi / 2;

% rate of actuator
% drone_params.r_sigma_a = 10 * pi / 2; % eta, x-axis
% drone_params.r_sigma_b = 10 * pi / 2; % xi, y-axis
drone_params.r_sigma_a = 5 * pi / 6; % eta, x-axis
drone_params.r_sigma_b = 5 * pi / 2; % xi, y-axis

drone_params.f_max = 1.5 * env_params.g;
drone_params.r_f = 50;

drone_params.sigma_w0 = 0;
drone_params.sigma_w = 400;
drone_params.r_sigma_w = 400;

beta_allo = [env_params.CT_u env_params.CT_l; env_params.prop_d * env_params.CP_u -env_params.prop_d * env_params.CP_l];
drone_params.prop_max = sqrt(beta_allo \ [drone_params.f_max; 0] / (env_params.rho * env_params.prop_d^4));

initial_state_x0 = get_initial_condition();
initial_state_x0 = Simulink.Parameter(initial_state_x0);

% drone_params.beta_allo = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];
% drone_params.prop_max = sqrt(beta_allo \ [f_max; 0] / (rho * prop_d^4));

%     key = {'g', 'rho', 'prop_d', 'CT_u', 'CT_l', 'CP_u', 'CP_l', 'm', 'm_a', 'm_fm', 'r_pg', 'r_fm', 'I_fm', 'I_a', 'I_b', 'mKp', 'mKd', 'pKp', ...
%             'Kp', 'Kv', 'Kq', 'Kw', ...
%             'Gamma_v', 'gamma_q', 'gamma_theta1', 'gamma_theta2', 'gamma_theta3', 'k_theta', ...
%             'delta_a', 'delta_b', 'v_w', 'C_d', 'I_xy', 'A_cs', 'esp_M', ...
%             'pos', 'psi', 'sigma_a', 'sigma_b', 'r_sigma_a', 'r_sigma_b', 'f_max', 'r_f', 'prop_max'};
%     value = {g, rho, prop_d, CT_u, CT_l, CP_u, CP_l, m, m_a, m_fm, r_pg, r_fm, I_fm, I_a, I_b, mKp, mKd, pKp, ...
%             Kp, Kv, Kq, Kw, ...
%             Gamma_v, gamma_q, gamma_theta1, gamma_theta2, gamma_theta3, k_theta, ...
%             delta_a, delta_b, v_w, C_d, I_xy, A_cs, esp_M, ...
%             pos, psi, sigma_a, sigma_b, r_sigma_a, r_sigma_b, f_max, r_f, prop_max};
%     params = containers.Map(key, value);
% end

function x0 = get_initial_condition()
    W0 = [0 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0.5, 0, 0.5), [9 1]);
    P0 = [2 2 1]';
    %P0 = [10 10 10]';
    dP0 = [0 0 0]';
    %dP0 = [3 3 3]';
    x0 = [W0; R0; dP0; P0];
end

function [pos, psi] = model_A9_con(GRID_SIZE)
    pos = [ 2  1 1  0 0 0 -1 -1 -2;
            0 -1 1 -2 0 2 -1  1  0;
            0  0 0  0 0 0  0  0  0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [0 0 0 0 0 0 0 0 0];
end

function [pos, psi] = model_A9_inc(GRID_SIZE)
    pos = [ 2  1 1  0 0 0 -1 -1 -2;
            0 -1 1 -2 0 2 -1  1  0;
            0  0 0  0 0 0  0  0  0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [0 pi/2 pi/2 0 pi/2 0 pi/2 pi/2 0];
end

function I_R_B = getI_R_B(psi, phi, theta)
    I_R_B = Rz(psi) * Rx(phi) * Ry(theta);
end
