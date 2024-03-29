% Default drone type
drone_type = "team";
% conf_name = "model_A9_con";
% conf_name = "model_A9_inc";

% Default task type
traj_type = "regulation";

% Controller sampling time
dt = 0.01;

env_params = gen_env_params();
drone_params = gen_drone_params(env_params, conf_name);
ctrl_params = gen_ctrl_params();
traj_params = gen_traj_params();

[initial_state_x0, initial_state_z0]= gen_initial_state(drone_params);


function env_params = gen_env_params()
    env_params.g = 9.818; % gravity
    env_params.rho = 1.225; % kg/m3
    env_params.prop_d = 0.0254 * 9; % 8 inch = 20.3 cm

    env_params.CT_u = 0.020231; % upper propeller thrust coefficient
    env_params.CT_l = 0.020231; % lower propeller thrust coefficient
    env_params.CP_u = 0.0188; % upper propeller drag coefficient
    env_params.CP_l = 0.0188; % lower propeller drag coefficient


    env_params.ext_wind = false;    % constant wind
    env_params.ext_v_w = [0; 0; 0];

    env_params.ext_dist = false;    % constant disturbance
    env_params.ext_dist_wrench = [0; 0; 0; 0; 0; 0];
    env_params.ext_load_pos = [0; 0; 0];
    env_params.ext_load_weight = 0;
end

function drone_params = gen_drone_params(env_params, conf_name)
    % Single Drone
    drone_params.m   = 0.7;
    drone_params.m_a = 0.15; % Mass, Kg
    drone_params.m_p = 0.0005; % Mass, Kg
    drone_params.m_fm = drone_params.m - drone_params.m_a;

    drone_params.m_uncertain   = 0.67434;
    drone_params.m_a_uncertain = 0.13048; % Mass, Kg
    drone_params.m_p_uncertain = 0.0005; % Mass, Kg
    drone_params.m_fm_uncertain = drone_params.m_uncertain - drone_params.m_a_uncertain;

    drone_params.r_pg = [0; 0; 0.026]; % Leverage length from c.p. to c.g.
    drone_params.r_fm = [0; 0; -6.237e-3]; % Leverage length from c.fm. to c.g.
    drone_params.l_pa = 0.0784;

    % region [inertia]
    % region [normal inertia]
    drone_params.I_fm = ...
        [0.003 0 0; 
        0 0.006 0; 
        0 0 0.003];

    drone_params.I_a = ...
        [0.0003       0       0; 
              0  0.0003       0;
              0       0  1.5e-5 ];        % Actuator Inertial

    drone_params.I_P = ...
        [4.0e-6 0 0; 
        0 4.0e-6 0; 
        0 0 7.0e-6]; % Propeller Inertial

    drone_params.I_b = ...
        [0.015   0    0; 
         0   0.003   -0; 
         0       0 0.02];        % Body Inertial
    % endregion [normal inertia]

    % region [uncertain inertia]
    drone_params.I_fm_uncertain = ...
        [0.0031 0 0; 
        0 0.0064 0; 
        0 0 0.0040];

    drone_params.I_a_uncertain = ...
        [0.000298      1e-7  -4.19e-6; 
        1e-7     0.0003036   -1.6e-7;
        -4.19e-6   -1.6e-7   1.537e-5 ];        % Actuator Inertial

    drone_params.I_P_uncertain = ...
        [4.01e-6 0 0; 
        0 3.05e-6 0; 
        0 0 7.05e-6]; % Propeller Inertial

    drone_params.I_b_uncertain = ...
        [0.016899   -5.25e-6    3.91e-6; 
        -5.25e-6 0.00275894 -0.0002391; 
        3.91e-6 -0.0002391 0.01794244];        % Body Inertial
    % endregion [uncertain inertia]
    % endregion [inertia]

    % Actuators and Motors
    drone_params.mKp = 200;
    drone_params.mKd = 20;
    drone_params.pKp = 1000;

    % Actuator Constraints
    drone_params.sigma_a = pi / 6;
    drone_params.sigma_b = pi / 2;
    drone_params.r_sigma_a = 50 * pi / 6; % eta, x-axis
    drone_params.r_sigma_b = 50 * pi / 2; % xi, y-axis

    % Motor Constraints
    drone_params.f_max = 1.5 * drone_params.m * env_params.g;
    drone_params.r_f = 50;

    drone_params.sigma_w0 = 0;
    drone_params.sigma_w = 400;
    drone_params.r_sigma_w = 40000;

    drone_params.beta_allo = [env_params.CT_u env_params.CT_l; -env_params.prop_d * env_params.CP_u env_params.prop_d * env_params.CP_l];
    drone_params.prop_max = sqrt(drone_params.beta_allo \ [drone_params.f_max; 0] / (env_params.rho * env_params.prop_d^4));
    drone_params.sigma_w = drone_params.prop_max(1);
    % drone_params.prop_max = [400; 400];

    % Team System
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
    elseif conf_name == "single_model"
        [pos, psi] = single_model(GRID_SIZE);
    elseif conf_name == "model_A4_inc_1_nav"
        [pos, psi] = model_A4_inc_1_nav(GRID_SIZE);
    end

    drone_params.pos = pos;
    drone_params.psi = psi;

    if size(pos, 2) > 1
        I_b = zeros(3);
        I_b_uncertain = zeros(3);
        for i = 1:length(pos)
            %x2 = pos(:, i).^2;
            %I_b = I_b + I_fm + m * diag([x2(2) + x2(3); x2(1) + x2(3); x2(2) + x2(1)]);
            I_b = I_b + drone_params.m * (pos(:, i)' * pos(:, i) * eye(3) - pos(:, i) * pos(:, i)');
            I_b_uncertain = I_b_uncertain + drone_params.m_uncertain * (pos(:, i)' * pos(:, i) * eye(3) - pos(:, i) * pos(:, i)');
            % drone_params.I_b + 
        end
        drone_params.I_bb = I_b;
        drone_params.I_bb_uncertain = I_b_uncertain;
        drone_params.mb = drone_params.m * size(pos, 2);
        drone_params.mb_uncertain = drone_params.m_uncertain * size(pos, 2);
    end 

    
    drone_params.agent_disable = false;
    drone_params.agent_disable_time = ... % start, end
        [10 15; 20, 25];
    drone_params.agent_disable_id = ...
        [2 3];

    % region [ext disturbance]
    drone_params.C_d = diag([1 1 0.5]);
    drone_params.I_xy = diag([1 1 0]);
    drone_params.A_cs = 0.114 * 10;
    drone_params.esp_M = 0.1;
    % endregion [ext disturbance]

    drone_params.model_uncertain = false;
end

function ctrl_params = gen_ctrl_params()
    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 5;
    ctrl_params.Kr = diag([1 1 1]) * 5;
    ctrl_params.Ko = diag([1 1 1]) * 10;

    ctrl_params.ext_ctrl = false;
    ctrl_params.ext_u_f = 0;
    ctrl_params.ext_u_t = 0;

    ctrl_params.ext_zctrl = false;
    ctrl_params.ext_z_d = 0;

    ctrl_params.ud_pypass = false;
    ctrl_params.zd_pypass = false;
    ctrl_params.internal_moment_bypass = false;
    ctrl_params.attitude_planner_non_neg_constraint = false;

    ctrl_params.force_projection = true;

    ctrl_params.pseudo_boundary = true;
    ctrl_params.pseudo_gamma = 0.1;

    ctrl_params.post_torque_enhance = true;

    ctrl_params.control_allocator = 1;

    % ctrl_params.control_allocator = "redistributed_nonlinear";
end

function traj_params = gen_traj_params()
    traj_params.amps = [1; 1; 1; 0; 0; 0];
    traj_params.phase = [0; 0; 0; 0; 0; 0];
    traj_params.freq = [0.5; 0; 0.5; 0; 0; 0];
    traj_params.offset = [0 0; 1 0; 0 0; 0 -pi/4; 0 0; 0 0];

    traj_params.sudden_change = false;
    traj_params.sudden_time_start = 0;
    traj_params.sudden_time_end = 0;
    traj_params.sudden_pos_offset = [0; 0; 0];
end

function [x0, z0] = gen_initial_state(drone_params)
    W0 = [0 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 0, 0), [9 1]);
    P0 = [0 0 0]';
    dP0 = [0 0 0]';
    x0 = [W0; R0; dP0; P0];

    z0 = reshape(ones([1 length(drone_params.psi)]) .* [0 0 0 0 100 100]', [6*length(drone_params.psi) 1]);
end

function [pos, psi] = single_model(GRID_SIZE)
    pos = [0;0;0];
    psi = 0;
end