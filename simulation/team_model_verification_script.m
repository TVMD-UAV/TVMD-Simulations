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

% Control Evaluations
num_cases = 13;
num_test = 6;
for k = 0:num_cases * num_test-1
    i = uint8(floor(k / num_cases) + 1);
    j = uint8(mod(k, num_cases) + 1);

    close all
    run('initialization/init_params_team.m')    
    model = 'SwarmSystem_2021b';
    simIn = Simulink.SimulationInput(model);

    if i == 1; [simIn, options, ctrl_params] = att_uncon_ud_bypass(simIn, ctrl_params);
    elseif i == 2; [simIn, options, ctrl_params] = att_uncon_zd_bypass(simIn, ctrl_params);
    elseif i == 3; [simIn, options, ctrl_params] = att_uncon_with_act_dyn(simIn, ctrl_params);
    elseif i == 4; [simIn, options, ctrl_params] = non_neg_ud_bypass(simIn, ctrl_params);
    elseif i == 5; [simIn, options, ctrl_params] = non_neg_zd_bypass(simIn, ctrl_params);
    elseif i == 6; [simIn, options, ctrl_params] = non_neg_with_act_dyn(simIn, ctrl_params);
    % elseif i == 7; [simIn, options, ctrl_params, initial_state_x0] = test(simIn, ctrl_params);
    end

    if j == 1; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting1(ctrl_params, options);
    elseif j == 2; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting2(ctrl_params, options);
    elseif j == 3; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting3(ctrl_params, options);
    elseif j == 4; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting4(ctrl_params, options);
    elseif j == 5; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting5(ctrl_params, options);
    elseif j == 6; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting6(ctrl_params, options);
    elseif j == 7; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting7(ctrl_params, options);
    elseif j == 8; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting8(ctrl_params, options);
    elseif j == 9; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting9(ctrl_params, options);
    elseif j == 10; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting10(ctrl_params, options);
    elseif j == 11; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting11(ctrl_params, options);
    elseif j == 12; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting12(ctrl_params, options);
    elseif j == 13; [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting13(ctrl_params, options);
    end
    matfilename = strcat(options('foldername'), options('filename'));
    % Simulation
    % try
    %     fprintf("%d, %s\n", k, matfilename)
    %     out = sim(simIn);
    %     save(matfilename, 'drone_params', 'env_params', 'ctrl_params', 'out', 'dt', 'initial_state_x0', 'initial_state_z0');
    %     plotter(env_params, drone_params, out, options);
    % catch
    %     fprintf("Failed to run sim %d\n", k)
    %     if isfile(strcat(matfilename, ".mat"))
    %         delete(strcat(matfilename, ".mat"))
    %     end
    % end
    % return


    % Load batched results
    projectname = split(options('foldername'), "\\");
    fprintf("%s & \t%s & ", projectname(end-1), options('filename'))
    if ~isfile(strcat(matfilename, ".mat"))
        fprintf("\n")
    else
        load(matfilename)
        extract_summary(env_params, drone_params, out)
        % extract_initial_conditions(initial_state_x0, initial_state_z0)
        % extract_gains(ctrl_params)
        % plotter(env_params, drone_params, out, options);
        % break
    end
end

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

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting1(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 0.2;
    ctrl_params.Kd = diag([1 1 1]) * 1;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [5 5 5]';
    R0 = reshape(eye(3) * getI_R_B(1, 1, 1), [9 1]);
    P0 = [10 10 -10]';
    dP0 = [10 10 10]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set1_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting2(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 0.2;
    ctrl_params.Kd = diag([1 1 1]) * 1;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [8 8 8]';
    R0 = reshape(eye(3) * getI_R_B(0.5, 0.5, 0.5), [9 1]);
    P0 = [5 5 5]';
    dP0 = [7 7 7]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set2_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting3(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 0.2;
    ctrl_params.Kd = diag([1 1 1]) * 1;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [-3 -5 8]';
    R0 = reshape(eye(3) * getI_R_B(0.5, 0.5, 0.5), [9 1]);
    P0 = [-5 10 5]';
    dP0 = [5 -5 7]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set3_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting4(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 0.2;
    ctrl_params.Kd = diag([1 1 1]) * 1;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [-3 -5 8]';
    R0 = reshape(eye(3) * getI_R_B(2, 2.5, 0.3), [9 1]);
    P0 = [-5 10 5]';
    dP0 = [5 -5 7]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set4_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting5(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 0.2;
    ctrl_params.Kd = diag([1 1 1]) * 1;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [-1 -2 3]';
    R0 = reshape(eye(3) * getI_R_B(2, 1.5, 0.3), [9 1]);
    P0 = [-10 10 -5]';
    dP0 = [5 5 -7]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set5_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting6(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 0.2;
    ctrl_params.Kd = diag([1 1 1]) * 1;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [-10 -8 3]';
    R0 = reshape(eye(3) * getI_R_B(2, 1.8, 0.3), [9 1]);
    P0 = [-10 15 -5]';
    dP0 = [8 10 -7]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set6_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting7(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 10;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [-10 -8 3]';
    R0 = reshape(eye(3) * getI_R_B(2, 0.3, 1.8), [9 1]);
    P0 = [-10 15 -5]';
    dP0 = [8 10 -7]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set7_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting8(ctrl_params, options)
    % initial angular velocity
    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 10;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [8 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 3, 0), [9 1]);
    P0 = [0 -10 0]';
    dP0 = [0 -10 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set8_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting9(ctrl_params, options)
    % Zero initial angular velocity
    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 10;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [0 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 3, 0), [9 1]);
    P0 = [0 -10 0]';
    dP0 = [0 -10 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set9_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting10(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 10;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [0 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 2.5, 0), [9 1]);
    P0 = [0 -10 0]';
    dP0 = [0 -10 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set10_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting11(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 10;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [8 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 2.5, 0), [9 1]);
    P0 = [0 -10 0]';
    dP0 = [0 -10 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set11_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting12(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 10;
    ctrl_params.Kr = diag([1 1 0.2]) * 10;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [0 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 3, 0), [9 1]);
    P0 = [0 -10 0]';
    dP0 = [0 -10 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set12_" + options('filename');
end

function [ctrl_params, initial_state_x0, options] = attitude_planner_evaluation_setting13(ctrl_params, options)
    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 10;
    ctrl_params.Kr = diag([1 1 0.2]) * 10;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [8 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 3, 0), [9 1]);
    P0 = [0 -10 0]';
    dP0 = [0 -10 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "set13_" + options('filename');
end

function [simIn, options, ctrl_params] = att_uncon_ud_bypass(simIn, ctrl_params)
    % Controller: full-pose, considers limited attainable space (using the estimated one)
    % Control Allocation: X
    % Actuator Constraints: X
    % Actuator Dynamcis: X, assuming u_d can always be fulfilled imdiately.
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_evaluation";
    projectname = "att_uncon_ud_bypass";
    filename = "eva_ctrl";
    ctrl_params.ud_pypass = true;
    ctrl_params.zd_pypass = false;
    ctrl_params.attitude_planner_non_neg_constraint = false;
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = att_uncon_zd_bypass(simIn, ctrl_params)
    % Controller: full-pose, considers limited attainable space (using the estimated one)
    % Control Allocation: Y
    % Actuator Constraints: Y
    % Actuator Dynamcis: X, assuming u_d can always be fulfilled imdiately.
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_evaluation";
    projectname = "att_uncon_zd_bypass";
    filename = "eva_ctrl";
    ctrl_params.ud_pypass = false;
    ctrl_params.zd_pypass = true;
    ctrl_params.attitude_planner_non_neg_constraint = false;
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = non_neg_ud_bypass(simIn, ctrl_params)
    % Controller: full-pose, considers limited attainable space (using the estimated one)
    % Control Allocation: X
    % Actuator Constraints: X
    % Actuator Dynamcis: X, assuming u_d can always be fulfilled imdiately.
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_evaluation";
    projectname = "non_neg_ud_bypass";
    filename = "eva_ctrl";
    ctrl_params.ud_pypass = true;
    ctrl_params.zd_pypass = false;
    ctrl_params.attitude_planner_non_neg_constraint = true;
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = non_neg_zd_bypass(simIn, ctrl_params)
    % Controller: full-pose, considers limited attainable space (using the estimated one)
    % Control Allocation: Y
    % Actuator Constraints: Y
    % Actuator Dynamcis: X, assuming u_d can always be fulfilled imdiately.
    % Attitude Planner: Constrain to be non-negative
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_evaluation";
    projectname = "non_neg_zd_bypass";
    filename = "eva_ctrl";
    ctrl_params.ud_pypass = false;
    ctrl_params.zd_pypass = true;
    ctrl_params.attitude_planner_non_neg_constraint = true;
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = att_uncon_with_act_dyn(simIn, ctrl_params)
    % Controller: full-pose, considers limited attainable space (using the estimated one)
    % Control Allocation: Y
    % Actuator Constraints: Y
    % Actuator Dynamcis: Y
    % Attitude Planner: Constrain to be non-negative
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_evaluation";
    projectname = "att_uncon_with_act_dyn";
    filename = "eva_ctrl";
    ctrl_params.ud_pypass = false;
    ctrl_params.zd_pypass = false;
    ctrl_params.attitude_planner_non_neg_constraint = false;

    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = non_neg_with_act_dyn(simIn, ctrl_params)
    % Controller: full-pose, considers limited attainable space (using the estimated one)
    % Control Allocation: Y
    % Actuator Constraints: Y
    % Actuator Dynamcis: Y
    % Attitude Planner: Constrain to be non-negative
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_evaluation";
    projectname = "non_neg_with_act_dyn";
    filename = "eva_ctrl";
    ctrl_params.ud_pypass = false;
    ctrl_params.zd_pypass = false;
    ctrl_params.attitude_planner_non_neg_constraint = true;

    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params, initial_state_x0] = test(simIn, ctrl_params)
    % Controller: full-pose, considers limited attainable space (using the estimated one)
    % Control Allocation: Y
    % Actuator Constraints: Y
    % Actuator Dynamcis: X, assuming u_d can always be fulfilled imdiately.
    % Attitude Planner: Constrain to be non-negative
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\allocation_gain_evaluation";
    projectname = "test";
    filename = "CA_gain";
    ctrl_params.ud_pypass = false;
    ctrl_params.zd_pypass = false;
    ctrl_params.attitude_planner_non_neg_constraint = true;

    ctrl_params.Kp = diag([1 1 1]) * 0.2;
    ctrl_params.Kd = diag([1 1 1]) * 1;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [-10 -8 3]';
    R0 = reshape(eye(3) * getI_R_B(2, 0.3, 1.8), [9 1]);
    P0 = [-10 15 -5]';
    dP0 = [8 10 -7]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params, initial_state_x0] = regulation_moving(simIn, ctrl_params)
    sim_time = 150;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));

    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\team_regulation";
    projectname = "moving";
    filename = "team_reg_moving";
    ctrl_params.ud_pypass = false;
    ctrl_params.zd_pypass = false;

    ctrl_params.Kp = diag([1 1 1]) * 10;
    ctrl_params.Kd = diag([1 1 1]) * 50;
    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 10;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;

    W0 = [5 5 5]';
    R0 = reshape(eye(3) * getI_R_B(0.5, 0.5, 0.5), [9 1]);
    R0 = reshape(eye(3) * getI_R_B(1, 1, 1), [9 1]);
    P0 = [10 10 10]';
    dP0 = [10 10 10]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, false);
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
    ctrl_params.Kr = diag([1 1 1]) * 5;
    ctrl_params.Ko = diag([1 1 1]) * 10;
    
    traj_params.amps = [5; 5; 1; 0.4; 0; 0];
    traj_params.phase = [0; pi/2; 0; 0; 0; 0];
    traj_params.freq = [0.5; 0.5; 0; 0.1; 0; 0];
    traj_params.offset = [0 0; 0 0; 0.5 0; 0 -pi/4; 0 0; 0 0];

    W0 = [0.1 0.1 0.1]';
    W0 = [1 1 1]';
    R0 = reshape(eye(3) * getI_R_B(0.2, 0.2, 0.2), [9 1]);
    P0 = [2 2 1]';
    dP0 = [0.5 0.5 0.5]';
    dP0 = [5 5 5]';
    initial_state_x0 = [W0; R0; dP0; P0];

    options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, false);
end

function plotter(env_params, drone_params, out, options)
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

    % Control
    u_f = squeeze(get(out.logsout, 'u_f').Values.Data);
    u_t = squeeze(get(out.logsout, 'u_t').Values.Data);
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


    % plot_team_animation(env_params, drone_params, 20, t, interp1(ts, p, t), interp1(ts, R, t), x_r, interp1(ts, zo, t), options);
    plot_team_3d_series(env_params, drone_params, [0, 1, 2, 3, 4, 5, 6, 20, 40], ts, p, R, zo, t, z_d, options);
    plot_attitude_on_unit_sphere(t, R_d(:, :, :), R(:, :, :), 10, options)
    plot_team_3d(env_params, drone_params, 20, ts, p, R, x_r, zo, options, false);
    plot_norm(t, eX, eV, eR, eOmega, options);
    plot_state(t, ts, p, v, x_r, omega, omega_d, eulZXY, attitude_d, options);
    plot_wrench(t, u_d, u, options)
    plot_torque(t, meta, options)
    plot_metrics(t, metrices(5, :), metrices(1, :), metrices(2, :), metrices(3, :), metrices(4, :), options);

    % plot_constraints_profile_with_rates(1, -r_sigma_x, r_sigma_x, t, zeros([1 length(t)]), ts, z(:, 2), "$$\dot{\eta}_x$$", options, false, [0 0], '_internal_rate_eta_x')
    % plot_constraints_profile_with_rates(1, -r_sigma_y, r_sigma_y, t, zeros([1 length(t)]), ts, z(:, 4), "$$\dot{\eta}_y$$", options, false, [400 0], '_internal_rate_eta_y')
    % plot_constraints_profile_with_rates(1, -sigma_x, sigma_x, t, z_d(1, :), ts, zo(:, 1), "$$\eta_x$$", options, false, [0 300], '_internal_eta_x')
    % plot_constraints_profile_with_rates(1, -sigma_y, sigma_y, t, z_d(2, :), ts, zo(:, 2), "$$\eta_y$$", options, false, [400 300], '_internal_eta_y')
    % plot_constraints_profile_with_rates(1, 0, sigma_w(1), t, z_d(3, :), ts, zo(:, 3), "$$\omega_{P1}$$", options, true, [0 600], '_internal_prop1')
    % plot_constraints_profile_with_rates(1, 0, sigma_w(2), t, z_d(4, :), ts, zo(:, 4), "$$\omega_{P2}$$", options, true, [400 600], '_internal_prop2')

    % plot_zstate_with_error(1, 0, sigma_w(1), t, z_d(3, :), ts, zo(:, 3), "$$\omega_{P1}$$", ["State", "(rad/s)"], ["Error", "(rad/s)"], options, true, [0 600], '_internal_prop1_error')
    % plot_zstate_with_error(1, 0, sigma_w(2), t, z_d(4, :), ts, zo(:, 4), "$$\omega_{P2}$$", ["State", "(rad/s)"], ["Error", "(rad/s)"], options, true, [400 600], '_internal_prop2_error')
    % plot_zstate_with_rate(1, -sigma_x, sigma_x, -r_sigma_x, r_sigma_x, t, z_d(1, :), ts, zo(:, 1), z(:, 2), "$$\eta_x$$", ["Angles", "(rad)"], ["Angle Rates", "(rad/s)"], options, false, [0 300], '_internal_eta_x_with_rate')
    % plot_zstate_with_rate(1, -sigma_y, sigma_y, -r_sigma_y, r_sigma_y, t, z_d(2, :), ts, zo(:, 2), z(:, 4), "$$\eta_y$$", ["Angles", "(rad)"], ["Angle Rates", "(rad/s)"], options, false, [400 300], '_internal_eta_y_with_rate')

    % Zoom in
    ttt = 10;
    [m, idx] = min(abs(t - ttt));
    t = t(1:idx);
    p = interp1(ts, p, t);
    v = interp1(ts, v, t);
    omega = interp1(ts, omega, t);
    eulZXY = interp1(ts, eulZXY, t);
    x_r = x_r(:, :, 1:idx);
    omega_d = omega_d(:, 1:idx);
    attitude_d = attitude_d(:, 1:idx);
    u_d = u_d(:, 1:idx);
    u = u(:, 1:idx);

    ts = interp1(ts, ts, t);

    options('filename') = options('filename') + "_zoomin_t10";
    plot_state(t, ts, p, v, x_r, omega, omega_d, eulZXY, attitude_d, options);
    plot_wrench(t, u_d, u, options)
end 

