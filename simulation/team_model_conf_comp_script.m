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

run_sim = false;
load_data = true;
record_data = true;
plot_batch = true;

% close all
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

% Control Evaluations
projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\icra2024\\team_conf";
conf_map = ["model_A1_con", "model_A2_con", "model_A3_con", "model_A4_con", "model_A5_con", "model_A6_con", "model_A7_con", "model_A8_con", "model_A9_con", "model_A10_con", ...
            "model_A1_inc", "model_A2_inc", "model_A3_inc", "model_A4_inc", "model_A5_inc", "model_A6_inc", "model_A7_inc", "model_A8_inc", "model_A9_inc", "model_A10_inc", ...
            "model_T10_con", "model_T10_inc"];
alg_map = ["erpi", "cgi", "sqp"];
num_test = 11;
num_conf = 20;
num_alg = 3;
num_init = 10;

if record_data == true
    num_entry = 5;
    record = zeros(num_conf, num_alg, num_init, num_entry);
end

if record_data || load_data || run_sim
for init_id = 1:num_init
    for i = 1:num_test
        for conf_id = 1:num_conf
            for alg_id = 1:num_alg

                % if i ~= 1; continue; end
                % if i >= 7; continue; end
                if conf_id <= 2; continue; end
                if conf_id > 10 && conf_id <= 12 ; continue; end
                
                % if alg_id ~= 1; continue; end
                % if conf_id ~= 10; continue; end
                % if i ~= 6 && i ~= 10; continue; end
                if i ~= 6; continue; end

                % if init_id == 2; continue; end
                % if init_id > 6; continue; end
                [gain_id, task_id, depre_init_id] = get_exp_ids_regulation();
                % [gain_id, task_id, depre_init_id] = get_exp_ids_tracking();
                
                conf_name = conf_map(conf_id);

                close all
                run('initialization/init_params_team.m')    
                model = 'SwarmSystem_2021b';
                simIn = Simulink.SimulationInput(model);

                % case selector
                if i == 1;     [simIn, options, drone_params]             = tracking_mot_failure(simIn, projectpath, conf_id, alg_id, drone_params);
                elseif i == 2; [simIn, options, env_params, drone_params] = tracking_ext_wind(simIn, projectpath, conf_id, alg_id, env_params, drone_params);
                elseif i == 3; [simIn, options, env_params]               = tracking_ext_dist(simIn, projectpath, conf_id, alg_id, env_params);
                elseif i == 4; [simIn, options, env_params, drone_params] = tracking_model_uncertain(simIn, projectpath, conf_id, alg_id, env_params, drone_params);
                elseif i == 5; [simIn, options, env_params, traj_params]  = tracking_traj_change(simIn, projectpath, conf_id, alg_id, env_params, traj_params);
                elseif i == 6; [simIn, options] = tracking_normal(simIn, projectpath, conf_id, alg_id);
            
                
                elseif i == 7; [simIn, options, drone_params, ctrl_params] = tracking_mot_failure_PTE_off(simIn, projectpath, conf_id, alg_id, drone_params, ctrl_params);
                elseif i == 8; [simIn, options, env_params, ctrl_params] = tracking_ext_dist_PTE_off(simIn, projectpath, conf_id, alg_id, env_params, ctrl_params);
                elseif i == 9; [simIn, options, env_params, traj_params, ctrl_params] = tracking_traj_change_PTE_off(simIn, projectpath, conf_id, alg_id, env_params, traj_params, ctrl_params);
                
                elseif i == 10; [simIn, options, ctrl_params] = tracking_normal_FPRJ_off(simIn, projectpath, conf_id, alg_id, ctrl_params);
                elseif i == 11; [simIn, options, drone_params, ctrl_params] = tracking_mot_failure_FPRJ_off(simIn, projectpath, conf_id, alg_id, drone_params, ctrl_params);
                end

                if gain_id == 1
                    [ctrl_params, options] = gain_setting1(ctrl_params, options);
                end

                if task_id == 1
                    [traj_type, traj_params, options] = traj_settings_sine_forward(traj_params, options);
                elseif task_id == 2
                    [traj_type, traj_params, options] = traj_settings_sine_upward(traj_params, options);
                elseif task_id == 3
                    [traj_type, traj_params, options] = traj_settings_regulation(traj_params, options);
                end 

                if init_id == 1
                    [initial_state_x0, options] = initial_setting1(options);
                elseif init_id == 2
                    [initial_state_x0, options] = initial_setting2(options);
                elseif init_id == 3
                    [initial_state_x0, options] = initial_setting3(options);
                elseif init_id == 4
                    [initial_state_x0, options] = initial_setting4(options);
                elseif init_id == 5
                    [initial_state_x0, options] = initial_setting5(options);
                elseif init_id == 6
                    [initial_state_x0, options] = initial_setting6(options);
                elseif init_id == 7
                    [initial_state_x0, options] = initial_setting7(options);
                elseif init_id == 8
                    [initial_state_x0, options] = initial_setting8(options);
                elseif init_id == 9
                    [initial_state_x0, options] = initial_setting9(options);
                elseif init_id == 10
                    [initial_state_x0, options] = initial_setting10(options);
                end
                
                if alg_id == 1
                    [ctrl_params, options] = allocator_erpi(ctrl_params, options);
                elseif alg_id == 2
                    [ctrl_params, options] = allocator_cgi(ctrl_params, options);
                elseif alg_id == 3
                    [ctrl_params, options] = allocator_interior(ctrl_params, options);
                else 
                    [ctrl_params, options] = allocator_erpi(ctrl_params, options);
                end

                matfilename = strcat(options('foldername'), options('filename'));
                if run_sim
                    fprintf("%d, %d, %d, %d, %s\n", i, conf_id, alg_id, init_id, matfilename)
                    % Simulation
                    try
                        out = sim(simIn);
                        save(matfilename, 'drone_params', 'env_params', 'ctrl_params', 'out', 'dt', 'initial_state_x0', 'initial_state_z0');
                        % plotter(env_params, drone_params, out, options);
                    catch
                        fprintf("Failed to run sim\n")
                        if isfile(strcat(matfilename, ".mat"))
                            delete(strcat(matfilename, ".mat"))
                        end
                    end

                end 
                if load_data
                    % Load batched results
                    projectname = split(options('foldername'), "\\");
                    fprintf("%d & \t%s & \t%s & \t%s & ", i, conf_map(conf_id), alg_map(alg_id), options('filename'))
                    if ~isfile(strcat(matfilename, ".mat"))
                        % Failure file
                        fprintf("\n")
                        if record_data == true
                            record(conf_id, alg_id, init_id, :) = ones([num_entry 1]) * nan;
                        end
                    else
                        load(matfilename)
                        % plotter(env_params, drone_params, out, options);
                        % extract_summary(env_params, drone_params, out);
                        % extract_initial_conditions(initial_state_x0, initial_state_z0)
                        % extract_gains(ctrl_params)
                        % plotter(env_params, drone_params, out, options);

                        if record_data == true
                            record(conf_id, alg_id, init_id, :) = extract_summary(env_params, drone_params, out);
                        end
                    end
                    % % return
                end 
            end
        end
    end
end
end

if plot_batch
    close all
    plot_batch_results(record, num_entry, num_init, num_alg)
end

function plot_batch_results(record, num_entry, num_init, num_alg)
    num_agent_ids = 3:10;

    % figure 
    % avr = mean(record(:, :, 1:9, :), 3);
    % ebrwa = squeeze(avr([num_agent_ids,num_agent_ids+10], 1, 1, :));
    % init_avr = reshape(ebrwa, [8, 2, num_entry]);
    % bar(num_agent_ids, squeeze(init_avr(:, :, 2)))

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % %% Differenet confuguration using EBRCA
    figure('Position', [810 10 400 300])
    subplot(2, 1, 1)
    %                       conf       alg     init     metrics
    x = {squeeze(record(num_agent_ids     , 1, :, 1))', ...
        squeeze(record(num_agent_ids + 10, 1, :, 1))'};
    ori_labels = {'Con', 'Inc'};
    num_labels = cellstr(string(num_agent_ids));
    % boxplot(squeeze(record(:, 1, :, 2))', g)
    % h = boxplotGroup(x,'primaryLabels',ori_labels,'SecondaryLabels',num_labels,'Colors','rk', 'Widths',0.8); hold on
    h = boxplotGroup(x, 'Colors','rk', 'Widths',0.8); hold on
    set_outlier_color(h);
    % h.axis.XTickLabelRotation = 90; 
    xpos = h.axis.Position;
    h.axis.Position = xpos + [0 0.05 0 -0.05];
    h.axis.XAxis.TickValues = [];
    % h.axis2.XAxis.TickLabelGapOffset = 8;
    % h.axis.FontName = 'Times New Roman';
    % h.axis2.FontName = 'Times New Roman';

    % scatter((1:8)*3-2,   squeeze(init_avr(:, 1, 2)), 'ro'); hold on
    % scatter((1:8)*3-1, squeeze(init_avr(:, 2, 2)), 'bo');
    % xl = xlabel('Configurations', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel('$\|\mathbf{e}_\mathbf{x}\|_2$ ST', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    % ylim([0 14])
    % xpos = xl.Position;
    % xl.Position = xpos + [0 -2 0];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(2, 1, 2)
    %                       conf       alg     init     metrics
    x = {squeeze(record(num_agent_ids     , 1, :, 2))', ...
        squeeze(record(num_agent_ids + 10, 1, :, 2))'};
        
    sum(isoutlier(cell2mat(x(1)), "median"), 1)
    sum(isoutlier(cell2mat(x(2)), "median"), 1)
    ori_labels = {'Con', 'Inc'};
    num_labels = cellstr(string(num_agent_ids));
    % boxplot(squeeze(record(:, 1, :, 2))', g)
    h = boxplotGroup(x,'primaryLabels',ori_labels,'SecondaryLabels',num_labels,'Colors','rk', 'Widths',0.8); hold on
    set_outlier_color(h);
    h.axis.XTickLabelRotation = 90; 
    xpos = h.axis2.Position;
    % h.axis2.Position = xpos + [0 0.15 0 -0.2];
    h.axis2.Position = xpos + [0 0.15 0 0];
    h.axis2.XAxis.TickLabelGapOffset = 8;
    h.axis.FontName = 'Times New Roman';
    h.axis2.FontName = 'Times New Roman';

    % scatter((1:8)*3-2,   squeeze(init_avr(:, 1, 2)), 'ro'); hold on
    % scatter((1:8)*3-1, squeeze(init_avr(:, 2, 2)), 'bo');
    xl = xlabel('Team Configurations', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel('$\|\mathbf{e}_\mathbf{x}\|_2$ ESS', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylim([0 0.8])
    xpos = xl.Position;
    xl.Position = xpos + [0 -0.2 0];
    xpos = h.axis2.Position;
    text(-2.8, -0.33, "#Agent", 'FontName', 'Times New Roman', 'FontSize', 10);

    foldername = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\icra2024\\team_conf";
    filename = "icra_batch";
    options = containers.Map({'savefig', 'savepdf', 'foldername', 'filename'}, ...
                             {true, false, foldername, filename});
    savefig_helper(options, '_conf_comp');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Differenet CA amonf orientations
    figure('Position', [410 10 400 300])
    subplot(2, 2, 1)
    % test1 = permute(record(num_agent_ids     , :, :, 2), [1, 3, 4, 2]);
    % test2 = reshape(test1, [8*num_init num_alg]);
    % mean(test1, [1, 2, 3])
    % mean(test2, 1)
    con = reshape(permute(record(num_agent_ids     , :, :, 1), [1, 3, 4, 2]), [8*num_init num_alg]);
    inc = reshape(permute(record(num_agent_ids + 10, :, :, 1), [1, 3, 4, 2]), [8*num_init num_alg]);

    %                       conf       alg     init     metrics
    x = {con, inc};
    ori_labels = {'Con', 'Inc'};
    num_labels = {'EBRCA', 'CGI', 'SQP'};
    % boxplot(squeeze(record(:, 1, :, 2))', g)
    % h = boxplotGroup(x,'primaryLabels',ori_labels,'SecondaryLabels',num_labels,'Colors','rk'); hold on
    h = boxplotGroup(x,'Colors','rk'); hold on
    set_outlier_color(h);
    xpos = h.axis.Position;
    h.axis.Position = xpos + [0 0.05 0 -0.05];
    h.axis.XAxis.TickValues = [];
    ylabel('$\mathbf{e}_\mathbf{x}$ ST', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylim([15 40])
    % xpos = xl.Position;
    % xl.Position = xpos + [0 -2 0];

    plot(h.axis.XLim, [18 18], 'k--'); hold on
    plot(h.axis.XLim, [22 22], 'k--'); hold on

    subplot(2, 2, 2)
    h = boxplotGroup(x,'Colors','rk'); hold on
    set_outlier_color(h);
    xpos = h.axis.Position;
    h.axis.Position = xpos + [0 0.05 0 -0.05];
    h.axis.XAxis.TickValues = [];
    ylim([18 22])

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Differenet CA amonf orientations
    subplot(2, 2, 3)
    % test1 = permute(record(num_agent_ids     , :, :, 2), [1, 3, 4, 2]);
    % test2 = reshape(test1, [8*num_init num_alg]);
    % mean(test1, [1, 2, 3])
    % mean(test2, 1)
    con = reshape(permute(record(num_agent_ids     , :, :, 2), [1, 3, 4, 2]), [8*num_init num_alg]);
    inc = reshape(permute(record(num_agent_ids + 10, :, :, 2), [1, 3, 4, 2]), [8*num_init num_alg]);

    %                       conf       alg     init     metrics
    x = {con, inc};
    sum(isoutlier(cell2mat(x(1)), "median"), 1)
    sum(isoutlier(cell2mat(x(2)), "median"), 1)

    xcon = squeeze(record(num_agent_ids     , :, :, 2));
    xinc = squeeze(record(num_agent_ids + 10, :, :, 2));
    
    sum(xcon > 1, [1 3])
    sum(xinc > 1, [1 3])

    sum(xcon > 10, [1 3])
    sum(xinc > 10, [1 3])

    ori_labels = {'Con', 'Inc'};
    num_labels = {'EBRCA', 'CGI', 'SQP'};
    % boxplot(squeeze(record(:, 1, :, 2))', g)
    h = boxplotGroup(x,'primaryLabels',ori_labels,'SecondaryLabels',num_labels,'Colors','rk'); hold on % ,'Whisker', 7
    set_outlier_color(h);


    h.axis.XTickLabelRotation = 90; 
    xpos = h.axis2.Position;
    % h.axis2.Position = xpos + [0 0.15 0 -0.2];
    % h.axis2.Position = xpos + [0 0.15 0 0];
    h.axis2.Position = xpos + [-0.023 0.15 0.023 0];
    h.axis2.XAxis.TickLabelGapOffset = 8;
    h.axis.FontName = 'Times New Roman';
    h.axis2.FontName = 'Times New Roman';

    % scatter((1:3)*3-2,   mean(con(1:72, :)), 'ro'); hold on
    % scatter((1:3)*3-1, mean(inc(1:72, :)), 'bo');
    % xl = xlabel('Control Allocation Algorithms', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel('$\mathbf{e}_\mathbf{x}$ ESS', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylim([0 0.12])
    % xpos = xl.Position;
    % xl.Position = xpos + [0 -0.16 0];
    % text(-2.8, -0.33, "CA", 'FontName', 'Times New Roman', 'FontSize', 10);

    plot(h.axis.XLim, [0.05 0.05], 'k--'); hold on
    plot(h.axis.XLim, [0.09 0.09], 'k--'); hold on

    %% 
    subplot(2, 2, 4)
    h = boxplotGroup(x,'primaryLabels',ori_labels,'SecondaryLabels',num_labels,'Colors','rk'); hold on
    set_outlier_color(h);
    h.axis.XTickLabelRotation = 90; 
    xpos = h.axis2.Position;
    h.axis2.Position = xpos + [-0.033 0.15 0.033 0];
    h.axis2.XAxis.TickLabelGapOffset = 8;
    h.axis.FontName = 'Times New Roman';
    h.axis2.FontName = 'Times New Roman';
    ylim([0.05 0.09])
    % xl = xlabel('Control Allocation Algorithms', 'FontName', 'Times New Roman', 'FontSize', 12);
    % xpos = xl.Position;
    % xl.Position = xpos + [0 -0.16 0];
    
    % set(gca,'Clipping','Off');
    % line([9.2 10.3], [0.77 1.32], 'Color', 'black', 'LineStyle' , '--');
    % line([9.2 10.3], [0.73 0.68], 'Color', 'black', 'LineStyle' , '--');
    annotation('line',[0.465 0.5],[0.72 0.92], 'linestyle','--', 'color','black', 'linewidth',1);
    annotation('line',[0.465 0.5],[0.67 0.63], 'linestyle','--', 'color','black', 'linewidth',1);

    annotation('line',[0.465 0.5],[0.52 0.59], 'linestyle','--', 'color','black', 'linewidth',1);
    annotation('line',[0.465 0.5],[0.42 0.31], 'linestyle','--', 'color','black', 'linewidth',1);

    
    foldername = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\icra2024\\team_conf";
    filename = "icra_batch";
    options = containers.Map({'savefig', 'savepdf', 'foldername', 'filename'}, ...
                             {true, false, foldername, filename});
    savefig_helper(options, '_ca');

    
    function set_outlier_color(h)
        outliers = findobj(h.boxplotGroup,'Tag','Outliers');
        outx = zeros([length(outliers), 1]);
        for m=1:length(outliers)
            outx(m) = outliers(m).XData(1);
        end
        [~, ind] = sort(outx);
        set(outliers(ind(2:2:length(outliers))), 'MarkerEdgeColor','k')
    end
end



function [gain_id, task_id, init_id] = get_exp_ids_regulation()
    gain_id = 1;
    task_id = 3;
    init_id = 2;
end

function [gain_id, task_id, init_id] = get_exp_ids_tracking()
    gain_id = 1;
    task_id = 1;
    init_id = 1;
end

% region [gain setting]
function [ctrl_params, options] = gain_setting1(ctrl_params, options)
    % ctrl_params.Kp = diag([1 0.5 1]) * 0.2;
    % ctrl_params.Kd = diag([1 0.5 1]) * 1.0;
    % ctrl_params.Kr = diag([5 1 1]) * 3;
    % ctrl_params.Ko = diag([5 1 1]) * 3;

    ctrl_params.Kp = diag([1 1 1]) * 2;
    ctrl_params.Kd = diag([1 1 1]) * 10;
    ctrl_params.Kr = diag([1 1 0.2]) * 5;
    ctrl_params.Ko = diag([1 1 0.5]) * 10;
    options('filename') = "gain1_" + options('filename');
end
% endregion [trajectory setting]

% region [trajectory setting]
function [traj_type, traj_params, options] = traj_settings_sine_forward(traj_params, options)
    traj_type = "sine";
    traj_params.amps = [2; 2; 1; 0; 0; 0];
    traj_params.freq = [1; 0; 1; 0; 0; 0];
    traj_params.offset = [0 0; 1 0; 0 0; 0 0; 0 -pi/6; 0 0];
    options('filename') = "sine_forward_" + options('filename');
end

function [traj_type, traj_params, options] = traj_settings_sine_upward(traj_params, options)
    traj_type = "sine";
    traj_params.amps = [5; 5; 1; 0.4; 0; 0];
    traj_params.phase = [0; pi/2; 0; 0; 0; 0];
    traj_params.freq = [0.5; 0.5; 0; 0.1; 0; 0];
    traj_params.offset = [0 0; 0 0; 0.5 0; -pi/8 0; 0 0; 0 0];
    options('filename') = "sine_upward_" + options('filename');
end

function [traj_type, traj_params, options] = traj_settings_regulation(traj_params, options)
    traj_type = "regulation";
    options('filename') = "regulation_" + options('filename');
end
% endregion [trajectory setting]

% region [initial setting]
function [initial_state_x0, options] = initial_setting1(options)
    W0 = [0.1 0.1 0.1]';
    R0 = reshape(eye(3) * getI_R_B(0.2, 0.2, 0.2), [9 1]);
    P0 = [2 2 1]';
    dP0 = [0.5 0.5 0.5]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init1_" + options('filename');
end

function [initial_state_x0, options] = initial_setting2(options)
    W0 = [0 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 2.5, 0), [9 1]);
    P0 = [0 -10 0]';
    dP0 = [0 -10 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init2_" + options('filename');
end

function [initial_state_x0, options] = initial_setting3(options)
    W0 = [0.1 0.1 0.1]';
    R0 = reshape(eye(3) * getI_R_B(0.5, 0.2, 0.2), [9 1]);
    P0 = [2 2 1]';
    dP0 = [2 2 2]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init3_" + options('filename');
end

function [initial_state_x0, options] = initial_setting4(options)
    W0 = [0.1 0.3 0.3]';
    R0 = reshape(eye(3) * getI_R_B(0.5, 0.2, 0.2), [9 1]);
    P0 = [2 2 1]';
    dP0 = [2 2 2]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init4_" + options('filename');
end

function [initial_state_x0, options] = initial_setting5(options)
    W0 = [0.1 0.3 0.3]';
    R0 = reshape(eye(3) * getI_R_B(0.5, 0.5, 0.2), [9 1]);
    P0 = [2 2 1]';
    dP0 = [2 2 2]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init5_" + options('filename');
end

function [initial_state_x0, options] = initial_setting6(options)
    W0 = [0.2 0.2 0.2]';
    R0 = reshape(eye(3) * getI_R_B(0.5, 1.2, 0.2), [9 1]);
    P0 = [2 2 1]';
    dP0 = [2 2 2]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init6_" + options('filename');
end

function [initial_state_x0, options] = initial_setting7(options)
    W0 = [0 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 2.5, 0.3), [9 1]);
    P0 = [2 -2 0]';
    dP0 = [0 -3 0]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init7_" + options('filename');
end

function [initial_state_x0, options] = initial_setting8(options)
    W0 = [0.3 0 0]';
    R0 = reshape(eye(3) * getI_R_B(0, 2.5, 0.3), [9 1]);
    P0 = [2 -2 0]';
    dP0 = [0 -3 3]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init8_" + options('filename');
end

function [initial_state_x0, options] = initial_setting9(options)
    W0 = [0.3 0.3 0]';
    R0 = reshape(eye(3) * getI_R_B(0.3, 2.5, 0.3), [9 1]);
    P0 = [2 -2 0]';
    dP0 = [0 -3 3]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init9_" + options('filename');
end

function [initial_state_x0, options] = initial_setting10(options)
    W0 = [0.3 0.3 0]';
    R0 = reshape(eye(3) * getI_R_B(0.3, 2.5, 1.5), [9 1]);
    P0 = [2 -2 0]';
    dP0 = [0 -3 3]';
    initial_state_x0 = [W0; R0; dP0; P0];
    options('filename') = "init10_" + options('filename');
end


% endregion [initial setting]

% region [Control allocator]
function [ctrl_params, options] = allocator_erpi(ctrl_params, options)
    ctrl_params.control_allocator = 1;
    options('filename') = "erpi_" + options('filename');
end

function [ctrl_params, options] = allocator_interior(ctrl_params, options)
    ctrl_params.control_allocator = 2;
    options('filename') = "interior_" + options('filename');
end

function [ctrl_params, options] = allocator_cgi(ctrl_params, options)
    ctrl_params.control_allocator = 3;
    options('filename') = "cgi_" + options('filename');
end
% endregion [Control allocator]


% region [Task setting]
function [simIn, options] = tracking_normal(simIn, projectpath, conf_id, alg_id)
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "normal";
    filename = "normal";

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, drone_params] = tracking_mot_failure(simIn, projectpath, conf_id, alg_id, drone_params)
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "motor_failure";
    filename = "mot_fail";

    drone_params.agent_disable = true;
    drone_params.agent_disable_time = ... % start, end
        [10 50; 100 100];
    if length(drone_params.psi) > 7
        drone_params.agent_disable_id = [uint8(2) uint8(5)];
    else
        drone_params.agent_disable_id = [uint8(2)];
    end

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, drone_params, ctrl_params] = tracking_mot_failure_PTE_off(simIn, projectpath, conf_id, alg_id, drone_params, ctrl_params)
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "motor_failure";
    filename = "mot_fail_PTE_off";

    drone_params.agent_disable = true;
    drone_params.agent_disable_time = ... % start, end
        [10 50; 100 100];
    drone_params.agent_disable_id = [uint8(2) uint8(5)];

    ctrl_params.post_torque_enhance = false;

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, env_params, drone_params] = tracking_ext_wind(simIn, projectpath, conf_id, alg_id, env_params, drone_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "ext_wind";
    filename = "ext_wind";

    env_params.ext_wind = true;
    env_params.ext_v_w = [3; 3; 0];

    drone_params.C_d = diag([0.1 0.1 0.05]);
    drone_params.I_xy = diag([1 1 0]);
    drone_params.A_cs = 0.05 * length(drone_params.psi);
    drone_params.esp_M = 0.1;

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, env_params] = tracking_ext_dist(simIn, projectpath, conf_id, alg_id, env_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "ext_dist";
    filename = "ext_dist";

    env_params.ext_dist = true;
    env_params.ext_dist_wrench = [0;0;0;0;0;0];
    env_params.ext_load_pos = [0.1; 0; -0.3];
    env_params.ext_load_weight = 2;

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, env_params, ctrl_params] = tracking_ext_dist_PTE_off(simIn, projectpath, conf_id, alg_id, env_params, ctrl_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "ext_dist";
    filename = "ext_dist_PTE_off";

    env_params.ext_dist = true;
    env_params.ext_dist_wrench = [0;0;0;0;0;0];
    env_params.ext_load_pos = [0.1; 0; -0.3];
    env_params.ext_load_weight = 2;

    ctrl_params.post_torque_enhance = false;

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, env_params, drone_params] = tracking_model_uncertain(simIn, projectpath, conf_id, alg_id, env_params, drone_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "model_uncertain";
    filename = "model_uncertain";

    drone_params.model_uncertain = true;

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, env_params, traj_params] = tracking_traj_change(simIn, projectpath, conf_id, alg_id, env_params, traj_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "traj_change";
    filename = "traj_change";

    traj_params.sudden_change = true;
    traj_params.sudden_time_start = 10;
    traj_params.sudden_time_end = 15;
    traj_params.sudden_pos_offset = [10; 0; 0];

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, env_params, traj_params, ctrl_params] = tracking_traj_change_PTE_off(simIn, projectpath, conf_id, alg_id, env_params, traj_params, ctrl_params)
    sim_time = 30;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "traj_change";
    filename = "traj_change_PTE_off";

    traj_params.sudden_change = true;
    traj_params.sudden_time_start = 10;
    traj_params.sudden_time_end = 15;
    traj_params.sudden_pos_offset = [10; 0; 0];

    ctrl_params.post_torque_enhance = false;

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, ctrl_params] = tracking_normal_FPRJ_off(simIn, projectpath, conf_id, alg_id, ctrl_params)
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "normal_FPRJ_off";
    filename = "normal_FPRJ_off";
    ctrl_params.force_projection = false;

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end

function [simIn, options, drone_params, ctrl_params] = tracking_mot_failure_FPRJ_off(simIn, projectpath, conf_id, alg_id, drone_params, ctrl_params)
    sim_time = 50;
    simIn = setModelParameter(simIn,"StopTime", string(sim_time));
    projectname = "motor_failure_FPRJ_off";
    filename = "mot_fail_FPRJ_off";
    
    ctrl_params.force_projection = false;

    drone_params.agent_disable = true;
    drone_params.agent_disable_time = ... % start, end
        [10 50; 100 100];
    if length(drone_params.psi) > 7
        drone_params.agent_disable_id = [uint8(2) uint8(5)];
    else
        drone_params.agent_disable_id = [uint8(2)];
    end

    options = gen_project_options_conf_alg(projectpath, projectname, filename, conf_id, alg_id, sim_time, false);
    % options = gen_project_options_subtask(projectpath, projectname, filename, sim_time, true);
end
% endregion [Task setting]


function out = extract_summary(env_params, drone_params, out)
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
    [pESS, pPeak, pST] = cal_response(t, eP);
    fprintf("\t%.2f & \t%.2f & \t%.2f & ", pESS, pPeak, pST)
    [vESS, peak, vST] = cal_response(t, eV);
    fprintf("\t%.2f & \t%.2f & \t%.2f & ", vESS, peak, vST)
    [RESS, peak, RST] = cal_response(t, eR);
    fprintf("\t%.2f & \t%.2f & \t%.2f & ", RESS, peak, RST)

    % Metrics
    % force error
    [avr, vmin, vmax, integ] = cal_statistic(t, metrics(1, :));
    fprintf("\t%.1f & \t%.1f & \t%.1f & ", avr, vmax, integ)
    % moment error
    [avr, vmin, vmax, integ] = cal_statistic(t, metrics(2, :));
    fprintf("\t%.1f & \t%.1f & \t%.1f & ", avr, vmax, integ)
    % force alignment
    [avr, vmin, vmax, integ] = cal_statistic(t, (metrics(3, :)+1)/2);
    fprintf("\t%.2f & \t%.2f & ", vmin, integ)
    % moment alignment
    [avr, vmin, vmax, integ] = cal_statistic(t, (metrics(4, :)+1)/2);
    fprintf("\t%.2f & \t%.2f ", vmin, integ)
    fprintf("\\\\ \n")

    out = [pST pESS RST RESS pPeak];
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
    increment = squeeze(get(out.recordout, 'increment').Values.Data);
    

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

    % plot_team_animation(env_params, drone_params, 20, t, interp1(ts, p, t), interp1(ts, R, t), x_r, interp1(ts, zo, t), options);
    % return
    plot_team_3d_series(env_params, drone_params, [0, 1, 2, 3, 4, 5, 6, 7], ts, p, R, zo, t, z_d, u_r_sat, [], [], options);
    ori_filename = options('filename');
    % options('filename') = ori_filename + "_t0";
    % plot_team_3d_series(env_params, drone_params, [0], ts, p, R, zo, t, z_d, u_r_sat, u_r, [0 -90 0], options);
    % options('filename') = ori_filename + "_t9_9";
    % plot_team_3d_series(env_params, drone_params, [9.9], ts, p, R, zo, t, z_d, u_r_sat, u_r, [0 -90 0], options);
    % options('filename') = ori_filename + "_t10_1";
    % plot_team_3d_series(env_params, drone_params, [10.1], ts, p, R, zo, t, z_d, u_r_sat, u_r, [0 -90 0], options);
    options('filename') = ori_filename;
    plot_attitude_on_unit_sphere(t, R_d(:, :, :), R(:, :, :), 10, options)
    % return
    plot_team_3d(env_params, drone_params, 0.3, ts, p, R, x_r, zo, options, false);
    options('savepdf') = false;
    
    % % return

    plot_norm(t, eX, eV, eR, eOmega, options);
    plot_state(t, ts, p, v, x_r, omega, omega_d, eulZXY, attitude_d, options);
    plot_wrench(t, u, u_d, u_r, options)
    plot_torque(t, meta, options)
    plot_force(t, meta, options)
    plot_metrics(t, metrices, increment, options);
    % return

    z_os = interp1(ts, zo, t)';
    C_p1 = kron(eye(n), [0 0 1 0]);
    C_p2 = kron(eye(n), [0 0 0 1]);
    C_x = kron(eye(n), [1 0 0 0]);
    C_y = kron(eye(n), [0 1 0 0]);
    % plot_constraints_profile_with_rates(1, -r_sigma_x, r_sigma_x, t, zeros([1 length(t)]), ts, z(:, 2), "$$\dot{\eta}_x$$", options, false, [0 0], '_internal_rate_eta_x')
    % plot_constraints_profile_with_rates(1, -r_sigma_y, r_sigma_y, t, zeros([1 length(t)]), ts, z(:, 4), "$$\dot{\eta}_y$$", options, false, [400 0], '_internal_rate_eta_y')
    plot_internal_state_profile(n, -sigma_x, sigma_x, t, C_x * z_d, C_x * z_os, "$$\eta_x$$", options, false, [0 300], [], '_internal_eta_x')
    plot_internal_state_profile(n, -sigma_y, sigma_y, t, C_y * z_d, C_y * z_os, "$$\eta_y$$", options, false, [400 300], [], '_internal_eta_y') % -0.1 0.1
    plot_internal_state_profile(n, 0, sigma_w(1), t, C_p1 * z_d, C_p1 * z_os, "$$\omega_{P1}$$", options, true, [0 600], [], '_internal_prop1')
    plot_internal_state_profile(n, 0, sigma_w(2), t, C_p2 * z_d, C_p2 * z_os, "$$\omega_{P2}$$", options, true, [400 600], [], '_internal_prop2')

    % plot_zstate_with_error(1, 0, sigma_w(1), t, z_d(3, :), ts, zo(:, 3), "$$\omega_{P1}$$", ["State", "(rad/s)"], ["Error", "(rad/s)"], options, true, [0 600], '_internal_prop1_error')
    % plot_zstate_with_error(1, 0, sigma_w(2), t, z_d(4, :), ts, zo(:, 4), "$$\omega_{P2}$$", ["State", "(rad/s)"], ["Error", "(rad/s)"], options, true, [400 600], '_internal_prop2_error')
    % plot_zstate_with_rate(1, -sigma_x, sigma_x, -r_sigma_x, r_sigma_x, t, z_d(1, :), ts, zo(:, 1), z(:, 2), "$$\eta_x$$", ["Angles", "(rad)"], ["Angle Rates", "(rad/s)"], options, false, [0 300], '_internal_eta_x_with_rate')
    % plot_zstate_with_rate(1, -sigma_y, sigma_y, -r_sigma_y, r_sigma_y, t, z_d(2, :), ts, zo(:, 2), z(:, 4), "$$\eta_y$$", ["Angles", "(rad)"], ["Angle Rates", "(rad/s)"], options, false, [400 300], '_internal_eta_y_with_rate')

    % ttt = 0.54
    % [m, idx1] = min(abs(ts - ttt));
    % [m, idx2] = min(abs(t - ttt));
    
    % R_r(:, :, idx2)
    % f_r(:, idx2)
    % plot_Rd_fr(R_r(:, :, idx2), R_d(:, :, idx2), R(idx1, :, :), u_f(:, idx2), theta(idx2), nb3(:, idx2), f_r(:, idx2), drone_params, env_params);
    
    
    % ttt = 0.56
    % [m, idx1] = min(abs(ts - ttt));
    % [m, idx2] = min(abs(t - ttt));

    % R_r(:, :, idx2)
    % f_r(:, idx2)
    % plot_Rd_fr(R_r(:, :, idx2), R_d(:, :, idx2), R(idx1, :, :), u_f(:, idx2), theta(idx2), nb3(:, idx2), f_r(:, idx2), drone_params, env_params);
    
    % ttt = 0.5
    % ttt = 0
    % [m, idx1] = min(abs(t - ttt));
    % ttt = 1
    % ttt = 2
    % [m, idx2] = min(abs(t - ttt));
    % idx1, idx2


    % Zoom in
    t_start = 9.5;
    t_end = 11;
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

    ts = interp1(ts, ts, t);

    options('labely_pos') = - (t_end - t_start) / 15 + t_start;
    options('filename') = options('filename') + "_zoomin_t10";
    plot_state(t, ts, p, v, x_r, omega, omega_d, eulZXY, attitude_d, options);
    plot_wrench(t, u, u_d, u_r, options)
    plot_torque(t, meta, options)
    plot_force(t, meta, options)
    plot_metrics(t, metrices, increment, options);
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
