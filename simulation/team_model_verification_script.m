addpath('control_allocation')  
addpath('control_allocation/allocation_helper')  
addpath('initialization')  
addpath('math_helper')  
addpath('model_func')  
addpath('math_helper')  
addpath('system_model')  
addpath('system_model/team_conf')  
addpath('visualization')  
addpath('visualization/viz_helper')  
close all
run('initialization/init_params_team.m')    
open_system('SwarmSystem_2021b');
model = 'SwarmSystem_2021b';
simIn = Simulink.SimulationInput(model);

% [simIn, options, ctrl_params] = thrust_verification(simIn, ctrl_params);
% [simIn, options, ctrl_params] = torque_verification(simIn, ctrl_params);
% [simIn, options, ctrl_params] = torque_x_verification(simIn, ctrl_params);
% [simIn, options, ctrl_params] = torque_y_verification(simIn, ctrl_params);
% [simIn, options, ctrl_params] = gyroscopic_verification(simIn, ctrl_params);
[simIn, options, ctrl_params, initial_state_x0] = regulation_hover(simIn, ctrl_params);
% [simIn, options, ctrl_params, initial_state_x0] = regulation_tilted(simIn, ctrl_params);
% [simIn, options, ctrl_params, initial_state_x0] = regulation_moving(simIn, ctrl_params);
% [simIn, options, ctrl_params, traj_params, initial_state_x0, traj_type] = tracking_sine_forward(simIn, ctrl_params, traj_params);
% [simIn, options, ctrl_params, traj_params, initial_state_x0, traj_type] = tracking_sine_upward(simIn, ctrl_params, traj_params);

out = sim(simIn);
matfilename = strcat(options('foldername'), options('filename'))
save(matfilename, 'drone_params', 'env_params', 'ctrl_params', 'out', 'dt', 'initial_state_x0', 'initial_state_z0');
plotter(drone_params, out, options);

function [simIn, options, ctrl_params] = thrust_verification(simIn, ctrl_params)
    sim_time = 0.1;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    ctrl_params.ext_ctrl = true;
    ctrl_params.ext_u_f = 1;
    ctrl_params.ext_u_t = 0;

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\single_verification";
    projectname = "thrust_verification_RNEA";
    filename = "sing_veri_thru";
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = torque_verification(simIn, ctrl_params)
    sim_time = 0.1;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    ctrl_params.ext_ctrl = true;
    ctrl_params.ext_u_f = 6.6207;
    ctrl_params.ext_u_t = [0; 0; 0.1];

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\single_verification";
    projectname = "torque_verification_RNEA";
    filename = "sing_veri_torq";
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = torque_x_verification(simIn, ctrl_params)
    sim_time = 1.0;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    ctrl_params.ext_ctrl = true;
    ctrl_params.ext_u_f = 6.6207;
    ctrl_params.ext_u_t = [0.01; 0; 0];

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\single_verification";
    projectname = "torque_x_verification_RNEA";
    filename = "sing_veri_torq_x";
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = torque_y_verification(simIn, ctrl_params)
    sim_time = 1.0;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    ctrl_params.ext_ctrl = true;
    ctrl_params.ext_u_f = 6.6207;
    ctrl_params.ext_u_t = [0; 0.01; 0];

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\single_verification";
    projectname = "torque_y_verification_RNEA";
    filename = "sing_veri_torq_y";
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = gyroscopic_verification(simIn, ctrl_params)
    sim_time = 1.0;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    ctrl_params.ext_zctrl = true;
    ctrl_params.ext_z_d = [0.1; 0; 213.153; 228.888]; % Desired angular velocity to gen Td=0.1

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\single_verification";
    projectname = "gyro_verification_RNEA";
    filename = "sing_veri_gyro";
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params, initial_state_x0] = regulation_hover(simIn, ctrl_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_regulation";
    projectname = "hovering";
    filename = "team_reg_hover";

    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 5;
    ctrl_params.Kr = diag([1 1 1]) * 30;
    ctrl_params.Ko = diag([1 1 1]) * 30;

    W0 = [0 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 0, 0), [9 1]);
    P0 = [2 2 1]';
    dP0 = [0 0 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, false);
end

function [simIn, options, ctrl_params, initial_state_x0] = regulation_tilted(simIn, ctrl_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_regulation";
    projectname = "tilted";
    filename = "team_reg_tilted";

    ctrl_params.Kp = diag([1 0.5 1]) * 0.2;
    ctrl_params.Kd = diag([1 0.5 1]) * 1.0;
    ctrl_params.Kr = diag([5 1 1]) * 3;
    ctrl_params.Ko = diag([5 1 1]) * 3;

W0 = [0 0 0]';
R0 = reshape(eye(3) * getI_R_B(0.2, 0.2, 0.2), [9 1]);
P0 = [2 2 1]';
dP0 = [0 0 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params, initial_state_x0] = regulation_moving(simIn, ctrl_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_regulation";
    projectname = "moving";
    filename = "team_reg_moving";

    ctrl_params.Kp = diag([1 0.5 1]) * 0.2;
    ctrl_params.Kd = diag([1 0.5 1]) * 1.0;
    ctrl_params.Kr = diag([5 1 1]) * 3;
    ctrl_params.Ko = diag([5 1 1]) * 3;

    W0 = [0.1 0.1 0.1]';
    R0 = reshape(eye(3) * getI_R_B(0.2, 0.2, 0.2), [9 1]);
    P0 = [2 2 1]';
    dP0 = [0.5 0.5 0.5]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params, traj_params, initial_state_x0, traj_type] = tracking_sine_forward(simIn, ctrl_params, traj_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_tracking";
    % projectname = "moving";
    % projectname = "tilted";
    projectname = "sine_forward";
    filename = "team_track_forward";

    traj_type = "sine";

    ctrl_params.Kp = diag([1 0.5 1]) * 0.2;
    ctrl_params.Kd = diag([1 0.5 1]) * 1.0;
    ctrl_params.Kr = diag([5 1 1]) * 3;
    ctrl_params.Ko = diag([5 1 1]) * 3;
    % ctrl_params.Kr = diag([5 1 1]) * 1;
    % ctrl_params.Ko = diag([5 1 1]) * 1;
    
    traj_params.amps = [1; 1; 1; 0; 0; 0];
    traj_params.freq = [0.5; 0; 0.5; 0; 0; 0];
    traj_params.offset = [0 0; 1 0; 0 0; 0 -pi/4; 0 0; 0 0];

    W0 = [0.1 0.1 0.1]';
    R0 = reshape(eye(3) * getI_R_B(0.2, 0.2, 0.2), [9 1]);
    P0 = [2 2 1]';
    dP0 = [0.5 0.5 0.5]';
    initial_state_x0 = [W0; R0; dP0; P0];

    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params, traj_params, initial_state_x0, traj_type] = tracking_sine_upward(simIn, ctrl_params, traj_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_tracking";
    projectname = "sine_upward";
    filename = "team_track_upward";

    traj_type = "sine";

    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 5;
    ctrl_params.Kr = diag([1 1 1]) * 30;
    ctrl_params.Ko = diag([1 1 1]) * 80;
    
    traj_params.amps = [5; 5; 1; 0.4; 0; 0];
    traj_params.phase = [0; pi/2; 0; 0; 0; 0];
    traj_params.freq = [0.5; 0.5; 0; 0.1; 0; 0];
    traj_params.offset = [0 0; 0 0; 0.5 0; 0 -pi/4; 0 0; 0 0];

    W0 = [0.1 0.1 0.1]';
    R0 = reshape(eye(3) * getI_R_B(0.2, 0.2, 0.2), [9 1]);
    P0 = [2 2 1]';
    dP0 = [0.5 0.5 0.5]';
    initial_state_x0 = [W0; R0; dP0; P0];

    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function plotter(drone_params, out, options)
    % region [Data Decoding]
    % States
    n = length(drone_params.psi);
    ts = out.tout;
    t = get(out.logsout, 'time').Values.Data;
    x = get(out.xout, 'x').Values.Data;
    z = get(out.xout, 'z').Values.Data;
    num_sample = length(ts);

    p = x(:, 16:18);
    v = x(:, 13:15);
    R = reshape(x(:, 4:12), [num_sample 3 3]); % 3x3
    omega = x(:, 1:3);
    eulZXY = rot2zxy_crossover(R);

    % Desired
    x_r = squeeze(get(out.logsout, 'x_r').Values.Data);
    attitude_d = squeeze(get(out.logsout, 'attitude_d').Values.Data);
    omega_d = zeros([3 length(t)]);
    z_d = squeeze(get(out.logsout, 'z_d').Values.Data);

    eTrans = get(out.logsout, 'eX').Values.Data;
    eX = squeeze(eTrans(:, 1, :));
    eV = squeeze(eTrans(:, 2, :));
    eR = squeeze(get(out.logsout, 'eR').Values.Data);
    eOmega = squeeze(get(out.logsout, 'eOmega').Values.Data);

    % meta
    meta = squeeze(get(out.logsout, 'ModelMetaData').Values.Data);
    % thrust = squeeze(get(out.logsout, 'thrust').Values.Data);

    % Control
    u_f = squeeze(get(out.logsout, 'u_f').Values.Data);
    u_t = squeeze(get(out.logsout, 'u_t').Values.Data);
    u = squeeze(get(out.logsout, 'output_wrench').Values.Data);

    B_M_f = squeeze(meta(:, 1, :));
    B_M_d = squeeze(meta(:, 2, :));
    B_M_g = squeeze(meta(:, 3, :));
    B_M_a = squeeze(meta(:, 4, :));
    B_M_delta = squeeze(meta(:, 5, :));

    u_d = [u_f; u_t];
    % end region [Data Decoding]

    % region [System Parameters]
    sigma_x = drone_params.sigma_a;
    sigma_y = drone_params.sigma_b;
    r_sigma_x = drone_params.r_sigma_a;
    r_sigma_y = drone_params.r_sigma_b;
    sigma_w = drone_params.prop_max;
    mKp = drone_params.mKp; mKd = drone_params.mKd; pKp = drone_params.pKp;
    % end region [System Parameters]

    [A_z, B_z, C_z] = gen_actuator_model(mKp, mKd, pKp);    
    zo = z * (kron(eye(n), C_z)');

    num_slot = 20;
    if t(end) * 1 < num_slot; num_slot = t(end) * 0.2; end
    interval = floor(num_sample / num_slot);
    fprintf("Ploting interval: %d\n", interval);
    r = 1:interval:num_sample;
    plot_3d(ts, r, p, p, x_r, u, R, options);

    plot_norm(t, eX, eV, eR, eOmega, options);
    plot_state(t, ts, p, v, x_r, omega, omega_d, eulZXY, attitude_d, options);
    plot_wrench(t, u_d, u, options)
    plot_torque(t, meta, options)

    % plot_constraints_profile_with_rates(1, -r_sigma_x, r_sigma_x, t, zeros([1 length(t)]), ts, z(:, 2), "$$\dot{\eta}_x$$", options, false, [0 0], '_internal_rate_eta_x')
    % plot_constraints_profile_with_rates(1, -r_sigma_y, r_sigma_y, t, zeros([1 length(t)]), ts, z(:, 4), "$$\dot{\eta}_y$$", options, false, [400 0], '_internal_rate_eta_y')
    % plot_constraints_profile_with_rates(1, -sigma_x, sigma_x, t, z_d(1, :), ts, zo(:, 1), "$$\eta_x$$", options, false, [0 300], '_internal_eta_x')
    % plot_constraints_profile_with_rates(1, -sigma_y, sigma_y, t, z_d(2, :), ts, zo(:, 2), "$$\eta_y$$", options, false, [400 300], '_internal_eta_y')
    % plot_constraints_profile_with_rates(1, 0, sigma_w(1), t, z_d(3, :), ts, zo(:, 3), "$$\omega_{P1}$$", options, true, [0 600], '_internal_prop1')
    % plot_constraints_profile_with_rates(1, 0, sigma_w(2), t, z_d(4, :), ts, zo(:, 4), "$$\omega_{P2}$$", options, true, [400 600], '_internal_prop2')

    plot_zstate_with_error(1, 0, sigma_w(1), t, z_d(3, :), ts, zo(:, 3), "$$\omega_{P1}$$", ["State", "(rad/s)"], ["Error", "(rad/s)"], options, true, [0 600], '_internal_prop1_error')
    plot_zstate_with_error(1, 0, sigma_w(2), t, z_d(4, :), ts, zo(:, 4), "$$\omega_{P2}$$", ["State", "(rad/s)"], ["Error", "(rad/s)"], options, true, [400 600], '_internal_prop2_error')
    plot_zstate_with_rate(1, -sigma_x, sigma_x, -r_sigma_x, r_sigma_x, t, z_d(1, :), ts, zo(:, 1), z(:, 2), "$$\eta_x$$", ["Angles", "(rad)"], ["Angle Rates", "(rad/s)"], options, false, [0 300], '_internal_eta_x_with_rate')
    plot_zstate_with_rate(1, -sigma_y, sigma_y, -r_sigma_y, r_sigma_y, t, z_d(2, :), ts, zo(:, 2), z(:, 4), "$$\eta_y$$", ["Angles", "(rad)"], ["Angle Rates", "(rad/s)"], options, false, [400 300], '_internal_eta_y_with_rate')
end 
