%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                    %
% Copyright 2024 Yen-Cheng Chu                                                       %
%                                                                                    %
% Licensed under the Apache License, Version 2.0 (the "License");                    %
% you may not use this file except in compliance with the License.                   %
% You may obtain a copy of the License at                                            %
%                                                                                    %
%     http://www.apache.org/licenses/LICENSE-2.0                                     %
%                                                                                    %
% Unless required by applicable law or agreed to in writing, software                %
% distributed under the License is distributed on an "AS IS" BASIS,                  %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.           %
% See the License for the specific language governing permissions and                %
% limitations under the License.                                                     %
%                                                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
open_system('SwarmSystem_2021b');


close all
% region [Single Simulation]
% To simulation a single case, please uncomment the following lines and run the script
% run('initialization/init_params_team.m')    
% model = 'SwarmSystem_2021b';
% simIn = Simulink.SimulationInput(model);
% % [simIn, options, ctrl_params, initial_state_x0] = test(simIn, ctrl_params);
% % [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting10(ctrl_params, options);
% %[simIn, options, drone_params, ctrl_params, traj_params, initial_state_x0, traj_type] = tracking_mot_failure(simIn, drone_params, ctrl_params, traj_params);
% [simIn, options, ctrl_params, traj_params, initial_state_x0, traj_type] = tracking_sine_forward(simIn, ctrl_params, traj_params);
% % [simIn, options, ctrl_params] = CA_test_without_pesudo_boundary(simIn, ctrl_params);
% % [simIn, options, ctrl_params] = CA_test_with_pesudo_boundary(simIn, ctrl_params);
% % Simulation
% out = sim(simIn);
% matfilename = strcat(options('foldername'), options('filename'));
% save(matfilename, 'drone_params', 'env_params', 'ctrl_params', 'out', 'dt', 'initial_state_x0', 'initial_state_z0');
% plotter(env_params, drone_params, out, options);
% return
% end region [Single Simulation]

% Control Evaluations

close all
conf_name = "model_A4_inc_1_nav";
% run('initialization/init_params_team.m')    
run('initialization/init_params_team_iros.m')    
model = 'SwarmSystem_2021b';
simIn = Simulink.SimulationInput(model);

[simIn, options] = tracking_normal(simIn);

[ctrl_params, options] = gain_setting1(ctrl_params, options);
[initial_state_x0, options] = initial_setting_stationary(options);
[ctrl_params, options] = allocator_erpi(ctrl_params, options);
[traj_type, traj_params, options] = traj_settings_hover_tilt(traj_params, options);

matfilename = strcat(options('foldername'), options('filename'));
% Simulation
% try
    fprintf("%s\n", matfilename)
    % out = sim(simIn);
    % save(matfilename, 'drone_params', 'env_params', 'ctrl_params', 'out', 'dt', 'initial_state_x0', 'initial_state_z0');
    plotter(env_params, drone_params, ctrl_params, out, options);
% catch
%     fprintf("Failed to run sim %d\n", k)
%     if isfile(strcat(matfilename, ".mat"))
%         delete(strcat(matfilename, ".mat"))
%     end
% end


% Load batched results
% projectname = split(options('foldername'), "\\");
% fprintf("%s & \t%s & ", projectname(end-1), options('filename'))
% if ~isfile(strcat(matfilename, ".mat"))
%     fprintf("\n")
% else
%     load(matfilename)
%     plotter(env_params, drone_params, out, options);
%     extract_summary(env_params, drone_params, out)
%     % extract_initial_conditions(initial_state_x0, initial_state_z0)
%     % extract_gains(ctrl_params)
%     % plotter(env_params, drone_params, out, options);
%     % break
% end
% return


% region [gain setting]
function [ctrl_params, options] = gain_setting1(ctrl_params, options)
    ctrl_params.Kp = diag([0.4 0.4 1]);
    ctrl_params.Kd = diag([0.8 0.8 2]);
    ctrl_params.Kr = diag([12 12 1]);
    ctrl_params.Ko = diag([8 8 1.5]);
    options('filename') = "gain1_" + options('filename');
end
% endregion [trajectory setting]

% region [trajectory setting]
function [traj_type, traj_params, options] = traj_settings_hover_tilt(traj_params, options)
    traj_type = "hover_tilt";
    options('filename') = "sine_forward_" + options('filename');
end
% endregion [trajectory setting]

% region [initial setting]
function [initial_state_x0, options] = initial_setting_stationary(options)
    W0 = [0 0 0]';
    R0 = reshape(eye(3), [9 1]);
    P0 = [0 0 0]';
    dP0 = [0 0 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "stationary_" + options('filename');
end
% endregion [initial setting]

% region [Control allocator]
function [ctrl_params, options] = allocator_erpi(ctrl_params, options)
    ctrl_params.control_allocator = 1;
    options('filename') = "erpi_" + options('filename');
end
% endregion [Control allocator]

% region [Task setting]
function [simIn, options] = tracking_normal(simIn)
    sim_time = 14;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\hover_tilt";
    projectname = "normal";
    filename = "normal";

    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end
% endregion [Task setting]


function extract_summary(env_params, drone_params, out)
    % States
    n = length(drone_params.psi);
    ts = out.tout;
    t = get(out.logsout, 'time').Values.Data;
    x = get(out.xout, 'x').Values.Data;
    num_sample = length(ts);

    p = x(:, 16:18);
    v = x(:, 13:15);
    R = reshape(x(:, 4:12), [num_sample 3 3]); % 3x3
    omega = x(:, 1:3);

    % Sampling 
    ps = interp1(ts, p, t)';
    vs = interp1(ts, v, t)';
    Rs = permute(interp1(ts, R, t), [2 3 1]);

    % Desired
    x_r = squeeze(get(out.logsout, 'x_r').Values.Data);
    R_r = squeeze(get(out.logsout, 'R_r').Values.Data);
    p_r = squeeze(x_r(:, 1, :));
    v_r = squeeze(x_r(:, 1, :));

    eRx = pagemtimes(R_r, 'transpose', Rs, 'none');% - pagemtimes(Rs, 'transpose', R_r, 'none');

    % eRx = 0.5 * (R_d' * R - R' * R_d);

    % calculate trace
    eR = 0.5 * (3 - [1 0 0 0 1 0 0 0 1] * reshape(eRx, [9 size(eRx, 3)]));
    
    eP = vecnorm(ps - p_r, 2, 1);
    eV = vecnorm(vs - v_r, 2, 1);
    
    metrics = squeeze(get(out.logsout, 'metrices').Values.Data);
    
    % Error
    [yfinal, peak, settling] = cal_response(t, eP);
    fprintf("\t%.2f & \t%.2f & \t%.2f & ", yfinal, peak, settling)
    [yfinal, peak, settling] = cal_response(t, eV);
    fprintf("\t%.2f & \t%.2f & \t%.2f & ", yfinal, peak, settling)
    [yfinal, peak, settling] = cal_response(t, eR);
    fprintf("\t%.2f & \t%.2f & \t%.2f & ", yfinal, peak, settling)

    % Metrics
    % force error
    [avr, vmin, vmax, integ] = cal_statistic(t, metrics(1, :));
    fprintf("\t%.1f & \t%.1f & \t%.1f & ", avr, vmax, integ)
    % moment error
    [avr, vmin, vmax, integ] = cal_statistic(t, metrics(2, :));
    fprintf("\t%.1f & \t%.1f & \t%.1f & ", avr, vmax, integ)
    % force alignment
    % [avr, vmin, vmax] = cal_statistic(t, (metrics(3, :)+1)/2);
    % moment alignment
    % fprintf("\t%.2f & ", avr)
    % [avr, vmin, vmax] = cal_statistic(t, (metrics(4, :)+1)/2);
    fprintf("\\\\ \n")

    
end

function extract_initial_conditions(initial_state_x0, initial_state_z0)
    W0 = initial_state_x0(1:3);
    R0 = rot2zxy(reshape(initial_state_x0(4:12), [3 3]));
    P0 = initial_state_x0(13:15);
    dP0 = initial_state_x0(16:18);
    fprintf("\t%.2f & ", [P0; dP0; R0'; W0])
    fprintf("\\\\ \n")
end

function extract_gains(ctrl_params)
    fprintf("\t%.1f & ", diag(ctrl_params.Kp))
    fprintf("\t%.1f & ", diag(ctrl_params.Kd))
    fprintf("\t%.1f & ", diag(ctrl_params.Kr))
    fprintf("\t%.1f & ", diag(ctrl_params.Ko))
    fprintf("\\\\ \n")
end


function plotter(env_params, drone_params, ctrl_params, out, options)
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
    eulZXY = wrapToPi(rot2zxy_crossover(R));

    % Desired
    x_r = squeeze(get(out.logsout, 'x_r').Values.Data);
    R_r = squeeze(get(out.logsout, 'R_r').Values.Data);
    R_d = squeeze(get(out.logsout, 'R_d').Values.Data);
    R_raw = squeeze(get(out.logsout, 'R').Values.Data);
    attitude_d = squeeze(get(out.logsout, 'attitude_d').Values.Data);
    omega_d = zeros([3 length(t)]);
    z_d = squeeze(get(out.logsout, 'z_d').Values.Data);
    metrices = squeeze(get(out.logsout, 'metrices').Values.Data);

    eTrans = get(out.logsout, 'eX').Values.Data;
    eX = squeeze(eTrans(:, 1, :));
    eV = squeeze(eTrans(:, 2, :));
    eR = squeeze(get(out.logsout, 'eR').Values.Data);
    eOmega = squeeze(get(out.logsout, 'eOmega').Values.Data);

    % meta
    meta = squeeze(get(out.logsout, 'ModelMetaData').Values.Data);
    % thrust = squeeze(get(out.logsout, 'thrust').Values.Data);
    increment = squeeze(get(out.recordout, 'increment').Values.Data);
    
    bounds = get(out.recordout, 'bounds').Values.Data;

    % Control
    u_f = squeeze(get(out.logsout, 'u_f').Values.Data);
    u_t = squeeze(get(out.logsout, 'u_t').Values.Data);
    u_r_sat = squeeze(get(out.logsout, 'f_r').Values.Data);
    u_r = squeeze(get(out.logsout, 'u_f_r').Values.Data);
    u = squeeze(get(out.logsout, 'output_wrench').Values.Data);

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

    theta = squeeze(get(out.logsout, 'theta').Values.Data);
    nb3 = squeeze(get(out.logsout, 'nb3').Values.Data);

    %
    eR_raw = zeros([1 length(t)]);
    size(R_d(:, :, 1))
    size(R_raw(:, :, 1))
    for i = 1:length(t)
        eR_raw(i) = norm(ctrl_params.Kr * (eye(3) - squeeze(R_d(:, :, i))' * squeeze(R_raw(:, :, i))));
    end

    z_os = interp1(ts, zo, t)';
    C_p1 = kron(eye(n), [0 0 1 0]);
    C_p2 = kron(eye(n), [0 0 0 1]);
    C_x = kron(eye(n), [1 0 0 0]);
    C_y = kron(eye(n), [0 1 0 0]);

    %%%
    % plot_animation_admissible_space(env_params, drone_params, 20, t, interp1(ts, p, t), interp1(ts, zo, t), options);
    % plot_team_animation(env_params, drone_params, 20, t, interp1(ts, p, t), interp1(ts, R, t), x_r, interp1(ts, zo, t), bounds, options, [20 20 20]', '_side');
    % plot_team_animation(env_params, drone_params, 20, t, interp1(ts, p, t), interp1(ts, R, t), x_r, interp1(ts, zo, t), bounds, options, [20 0 0]', '_x');
    % plot_team_animation(env_params, drone_params, 20, t, interp1(ts, p, t), interp1(ts, R, t), x_r, interp1(ts, zo, t), bounds, options, [0 20 0]', '_y');
    % plot_team_animation(env_params, drone_params, 20, t, interp1(ts, p, t), interp1(ts, R, t), x_r, interp1(ts, zo, t), bounds, options, [0 0 20]', '_z');
    % plot_profile_animation(env_params, drone_params, 20, t, interp1(ts, p, t), interp1(ts, R, t), x_r, eX, eR, increment, metrices, options)
    % plot_internal_animation(env_params, drone_params, 20, t, interp1(ts, zo, t)', options)
    % % return
    % plot_team_3d_series(env_params, drone_params, [0, 1, 2, 4, 6, 7, 8, 10, 12, 13, 14], ts, p, R, zo, t, z_d, u_r_sat, [], [90 0 0], options);
    % % return
    % % ori_filename = options('filename');
    % % options('filename') = ori_filename + "_t0";
    % % plot_team_3d_series(env_params, drone_params, [0], ts, p, R, zo, t, z_d, u_r_sat, u_r, [0 -90 0], options);
    % % plot_attitude_on_unit_sphere(t, R_d(:, :, :), R(:, :, :), 10, options)
    % % % % return
    % % % plot_team_3d(env_params, drone_params, 0.3, ts, p, R, x_r, zo, options, true);
    % % % options('savepdf') = false;

    % % frame = getframe(gcf);
    % % im = frame2im(frame);
    % % [imind, cm] = rgb2ind(im, 256);
    % % imwrite(imind, cm, 'initial.png', 'png')
    % set(gcf,'Color',[1, 1, 1])
    % set(gca,'Color',[1, 1, 1])
    % set(groot,{'DefaultAxesXColor','DefaultAxesYColor','DefaultAxesZColor', 'defaultfigurecolor'}, ...
    %     {[0.1500 0.1500 0.1500], [0.1500 0.1500 0.1500], [0.1500 0.1500 0.1500], [0.9400 0.9400 0.9400]})
    
    % % return

    
    % plot_lyapunov_candidates(t, ctrl_params.Kp*eX, ctrl_params.Kd*eV, eR_raw, ctrl_params.Ko*eOmega, options);
    % plot_norm(t, eX, eV, eR, eOmega, options);
    % plot_state(t, ts, p, v, x_r, omega, omega_d, eulZXY, attitude_d, options);
    % plot_wrench(t, u, u_d, u_r, options)
    % plot_torque(t, meta, options)
    % plot_force(t, meta, options)
    % plot_metrics(t, metrices, increment, options);
    % % % return

    % % plot_constraints_profile_with_rates(1, -r_sigma_x, r_sigma_x, t, zeros([1 length(t)]), ts, z(:, 2), "$$\dot{\eta}_x$$", options, false, [0 0], '_internal_rate_eta_x')
    % % plot_constraints_profile_with_rates(1, -r_sigma_y, r_sigma_y, t, zeros([1 length(t)]), ts, z(:, 4), "$$\dot{\eta}_y$$", options, false, [400 0], '_internal_rate_eta_y')
    % % figure('Position', [1410 10 400 300])
    % figure('Position', [1410 10 500 250])
    % plot_internal_state_profile(n, -sigma_x, sigma_x, t, C_x * z_d, C_x * z_os, "$$\eta_x$$", options, false, [0 300], [], '_internal_eta_x', 1)
    % plot_internal_state_profile(n, -sigma_y, sigma_y, t, C_y * z_d, C_y * z_os, "$$\eta_y$$", options, false, [400 300], [], '_internal_eta_y', 2) % -0.1 0.1
    % plot_internal_state_profile(n, 0, sigma_w(1), t, C_p1 * z_d, C_p1 * z_os, "$$\omega_{P1}$$", options, true, [0 600], [], '_internal_prop1', 3)
    % plot_internal_state_profile(n, 0, sigma_w(2), t, C_p2 * z_d, C_p2 * z_os, "$$\omega_{P2}$$", options, true, [400 600], [], '_internal_prop2', 4)

    % % plot_zstate_with_error(1, 0, sigma_w(1), t, z_d(3, :), ts, zo(:, 3), "$$\omega_{P1}$$", ["State", "(rad/s)"], ["Error", "(rad/s)"], options, true, [0 600], '_internal_prop1_error')
    % % plot_zstate_with_error(1, 0, sigma_w(2), t, z_d(4, :), ts, zo(:, 4), "$$\omega_{P2}$$", ["State", "(rad/s)"], ["Error", "(rad/s)"], options, true, [400 600], '_internal_prop2_error')
    % % plot_zstate_with_rate(1, -sigma_x, sigma_x, -r_sigma_x, r_sigma_x, t, z_d(1, :), ts, zo(:, 1), z(:, 2), "$$\eta_x$$", ["Angles", "(rad)"], ["Angle Rates", "(rad/s)"], options, false, [0 300], '_internal_eta_x_with_rate')
    % % plot_zstate_with_rate(1, -sigma_y, sigma_y, -r_sigma_y, r_sigma_y, t, z_d(2, :), ts, zo(:, 2), z(:, 4), "$$\eta_y$$", ["Angles", "(rad)"], ["Angle Rates", "(rad/s)"], options, false, [400 300], '_internal_eta_y_with_rate')

    % % ttt = 0.54
    % % [m, idx1] = min(abs(ts - ttt));
    % % [m, idx2] = min(abs(t - ttt));
    
    % % R_r(:, :, idx2)
    % % f_r(:, idx2)
    % % plot_Rd_fr(R_r(:, :, idx2), R_d(:, :, idx2), R(idx1, :, :), u_f(:, idx2), theta(idx2), nb3(:, idx2), u_r_sat(:, idx2), drone_params, env_params);
    
    
    % % ttt = 0.56
    % % [m, idx1] = min(abs(ts - ttt));
    % % [m, idx2] = min(abs(t - ttt));

    % % R_r(:, :, idx2)
    % % f_r(:, idx2)
    % % plot_Rd_fr(R_r(:, :, idx2), R_d(:, :, idx2), R(idx1, :, :), u_f(:, idx2), theta(idx2), nb3(:, idx2), f_r(:, idx2), drone_params, env_params);
    
    % % ttt = 0.5
    % % ttt = 0
    % % [m, idx1] = min(abs(t - ttt));
    % % ttt = 1
    % % ttt = 2
    % % [m, idx2] = min(abs(t - ttt));
    % % idx1, idx2


    % Zoom in
    t_start = 0;
    t_end = 0.1;
    [m, idx_start] = min(abs(t - t_start));
    [m, idx_end] = min(abs(t - t_end));
    t = t(idx_start:idx_end);
    p = interp1(ts, p, t);
    v = interp1(ts, v, t);
    omega = interp1(ts, omega, t);
    eulZXY = interp1(ts, eulZXY, t);
    x_r = x_r(:, :, idx_start:idx_end);
    omega_d = omega_d(:, idx_start:idx_end);
    attitude_d = attitude_d(:, idx_start:idx_end);
    u_d = u_d(:, idx_start:idx_end);
    u_r_sat = u_r_sat(:, idx_start:idx_end);
    u_r = u_r(:, idx_start:idx_end);
    u = u(:, idx_start:idx_end);
    meta = meta(:, :, idx_start:idx_end);
    metrices = metrices(:, idx_start:idx_end);
    increment = increment(:, idx_start:idx_end);
    z_os = z_os(:, idx_start:idx_end);

    ts = interp1(ts, ts, t);

    options('labely_pos') = - (t_end - t_start) / 15 + t_start;
    options('filename') = options('filename') + "_zoomin_t0_1";
    plot_state(t, ts, p, v, x_r, omega, omega_d, eulZXY, attitude_d, options);
    plot_wrench(t, u, u_d, u_r, options)
    plot_torque(t, meta, options)
    plot_force(t, meta, options)
    plot_metrics(t, metrices, increment, options);

    figure('Position', [1410 10 500 250])
    plot_internal_state_profile(n, -sigma_x, sigma_x, t, C_x * z_d, C_x * z_os, "$$\eta_x$$", options, false, [0 300], [], '_internal_eta_x', 1)
    plot_internal_state_profile(n, -sigma_y, sigma_y, t, C_y * z_d, C_y * z_os, "$$\eta_y$$", options, false, [400 300], [], '_internal_eta_y', 2) % -0.1 0.1
    plot_internal_state_profile(n, 0, sigma_w(1), t, C_p1 * z_d, C_p1 * z_os, "$$\omega_{P1}$$", options, true, [0 600], [], '_internal_prop1', 3)
    plot_internal_state_profile(n, 0, sigma_w(2), t, C_p2 * z_d, C_p2 * z_os, "$$\omega_{P2}$$", options, true, [400 600], [], '_internal_prop2', 4)

end 


function plot_Rd_fr(Rr, R_d, R, u_f, theta, nb3, f_rr, drone_params, env_params)
    % Rr, R, u_f
    Rr = squeeze(Rr);
    R_d = squeeze(R_d);
    R = squeeze(R);
    u_f = squeeze(u_f);
    theta = squeeze(theta);
    f_r = 2 * R * u_f / norm(u_f);
    % f_rr = 2 * f_rr / norm(f_rr);
    figure

    rd3 = Rr(:, 3);
    k = cross(rd3, f_rr) / norm(cross(rd3, f_rr));
    % theta
    nb3
    out = rot_vec_by_theta(rd3, k, theta);
    quiver3(0, 0, 0, out(1), out(2), out(3), 'magenta', 'DisplayName', 'rotated rd'); hold on
    quiver3(0, 0, 0, nb3(1), nb3(2), nb3(3), 'black', 'LineWidth', 2, 'DisplayName', 'nb3'); hold on
    
    quiver3(0, 0, 0, f_r(1), f_r(2), f_r(3), 'red', 'DisplayName', '$f_r$'); hold on
    quiver3(0, 0, 0, f_rr(1), f_rr(2), f_rr(3), 'red', 'LineStyle', '--', 'DisplayName', '$f_{rr}$'); hold on
    % quiver3(0, 0, 0, 5*nb3(1), 5*nb3(2), 5*nb3(3), 'Color', '#EDB120', 'DisplayName', "$b_{3}'$"); hold on
    % quiver3(0, 0, 0, 5*b3r(1), 5*b3r(2), 5*b3r(3), 'Color', '#7E2F8E', 'DisplayName', '$b_{3r}$'); hold on

    quiver3(0, 0, 0, Rr(1, 1), Rr(2, 1), Rr(3, 1), 'red', 'LineWidth', 2, 'DisplayName', "$\mathbf{R_r}_1'$"); hold on
    quiver3(0, 0, 0, Rr(1, 2), Rr(2, 2), Rr(3, 2), 'green', 'LineWidth', 2, 'DisplayName', "$\mathbf{R_r}_2'$"); hold on
    quiver3(0, 0, 0, Rr(1, 3), Rr(2, 3), Rr(3, 3), 'blue', 'LineWidth', 2, 'DisplayName', "$\mathbf{R_r}_3'$"); hold on
    
    quiver3(0, 0, 0, R(1, 1), R(2, 1), R(3, 1), 'red', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', "$\mathbf{b}_1'$"); hold on
    quiver3(0, 0, 0, R(1, 2), R(2, 2), R(3, 2), 'green', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', "$\mathbf{b}_2'$"); hold on
    quiver3(0, 0, 0, R(1, 3), R(2, 3), R(3, 3), 'blue', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', "$\mathbf{b}_3'$"); hold on
    
    quiver3(0, 0, 0, R_d(1, 1), R_d(2, 1), R_d(3, 1), 'red', 'LineStyle', ':', 'LineWidth', 2, 'DisplayName', "$\mathbf{R_d}_1'$"); hold on
    quiver3(0, 0, 0, R_d(1, 2), R_d(2, 2), R_d(3, 2), 'green', 'LineStyle', ':', 'LineWidth', 2, 'DisplayName', "$\mathbf{R_d}_2'$"); hold on
    quiver3(0, 0, 0, R_d(1, 3), R_d(2, 3), R_d(3, 3), 'blue', 'LineStyle', ':', 'LineWidth', 2, 'DisplayName', "$\mathbf{R_d}_3'$"); hold on
    legend('Interpreter', 'latex')
    plot_est_boundary_elliptic_cone(drone_params, R_d)
    % plot_est_boundary(R_d, drone_params, env_params)

    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
end

function out = rot_vec_by_theta(vec, rot_axis, theta)
    out = vec * cos(theta) + cross(rot_axis, vec) * sin(theta) + rot_axis * (rot_axis' * vec) * (1 - cos(theta));
end
