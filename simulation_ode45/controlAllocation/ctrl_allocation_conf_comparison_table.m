addpath('../helper_functions')
addpath('../model/model')
addpath('../model/model/swarm_conf')

% projectpath = 'H:\\我的雲端硬碟\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\outputs\\230220_fullpose_high_gains\\';

projectpath = 'H:\\我的雲端硬碟\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\outputs\\';

projectname = "230223_conf_comparison";
conf_map = ["model_A3_con", "model_A4_con", "model_A5_con", "model_A6_con", "model_A7_con", "model_A8_con", "model_A9_con", "model_A10_con", ...
            "model_A3_inc", "model_A4_inc", "model_A5_inc", "model_A6_inc", "model_A7_inc", "model_A8_inc", "model_A9_inc", "model_A10_inc", ...
            "model_T10_con", "model_T10_inc"];
alg_map = ["moore", "nullspace", "redistributed", "null_redistributed"];

projectpath = strcat(projectpath, projectname, "\\");

% conf_foldername = ["tri2conf\\", "trifat2Conf\\", "slim2conf\\", "fat2conf\\"];
% ctrl_foldername = ["moore\\", "redistributed\\", "null_redistributed\\"];

conf_legends = ["A3-Con", "A4-Con", "A5-Con", "A6-Con", "A7-Con", "A8-Con", "A9-Con", "A10-Con", ...
                "A3-Inc", "A4-Inc", "A5-Inc", "A6-Inc", "A7-Inc", "A8-Inc", "A9-Inc", "A10-Inc", ...
                "T10-Con", "T10-Inc"];
ctrl_legends = [" Moore", "Nullspace", " RPI", " NRPI"];

filename = 'swarm_allocation.mat';
cons = reshape([1:8; 9:16], [16 1]);


for i = 1:16
    for alg_id = 1:3
        conf_id = cons(i);
        conf_name = conf_map(conf_id);
        alg_name = alg_map(alg_id);
        foldername = strcat(conf_name, "\\", alg_name, "\\");
        project_full_dirname = strcat(projectpath, foldername);

        full_path = strcat(project_full_dirname, filename);
        legend_name = strcat(conf_legends(conf_id), "&", ctrl_legends(alg_id));
        
        [key, params] = get_swarm_params(conf_name);
        n = length(params('psi'));

        if ~isfile(full_path)
            fprintf("\n%s\t & ", legend_name)
            fprintf("-&-&-&-&-&-&-&-&-&-&-&-\\\\")
        else
            load(full_path)

            dP = y(:, 13:15);
            P = y(:, 16:18);

            traj = reshape(refs(:, 1:12), [length(y), 3, 4]);
            traj = permute(traj, [1, 3, 2]);

            eR = outputs(:, 16:18);
            eOmega = outputs(:, 19:21);

            norm_P = vecnorm(traj(:, 1:3, 1) - P, 2, 2);
            norm_V = vecnorm(traj(:, 1:3, 2) - dP, 2, 2);
            norm_R = vecnorm(eR, 2, 2);
            norm_Omega = vecnorm(eOmega, 2, 2);

            fprintf("\n%s\t & ", legend_name)
            [yfinal, peak, settling] = cal_response(t, norm_P);
            if (yfinal > 1)
                fprintf("-&-&-&-&-&-&-&-&-&-&-&-\\\\")
            else
                fprintf("\t%.2f & \t%.2f & \t%.2f & ", yfinal, peak, settling)
                % [yfinal, peak, settling] = cal_response(t, norm_V);
                % fprintf("\t%.2f & \t%.2f & \t%.2f & \t%.2f & ", yfinal, peak, settling)
                [yfinal, peak, settling] = cal_response(t, norm_R);
                fprintf("\t%.2f & \t%.2f & \t%.2f & ", yfinal, peak, settling)
                % [yfinal, peak, settling] = cal_response(t, norm_Omega);
                % fprintf("\t%.2f & \t%.2f & \t%.2f & \t%.2f & ", yfinal, peak, settling, rising)

                [avr, vmin, vmax] = cal_statistic(t, metrics(:, 2));
                fprintf("\t%.1f & \t%.1f & ", avr, vmax)
                [avr, vmin, vmax] = cal_statistic(t, metrics(:, 3));
                fprintf("\t%.1f & \t%.1f & ", avr, vmax)
                [avr, vmin, vmax] = cal_statistic(t, (metrics(:, 4)+1)/2);
                fprintf("\t%.2f & ", avr)
                [avr, vmin, vmax] = cal_statistic(t, (metrics(:, 5)+1)/2);
                fprintf("\t%.2f \\\\ ", avr)
                % [avr, vmin, vmax] = cal_statistic(t, metrics(:, 1));
                % fprintf("\t%.2f & \t%.2f \\\\ ", vmin, avr)
            end
        end
    end
    fprintf("\n\\hline")
end

function [yfinal, peak, settling] = cal_response(t, d)
    % S = stepinfo(d,t, 0);
    yfinal = mean(d(length(d)-10:end));
    % peak = S.Peak;
    peak = max(d);
    % settling = S.SettlingTime;
    settle_range = 0.03 * peak; 
    mask = (d > settle_range);
    mask(1:10) = 1;
    t(mask) = inf;
    [~, settling_i] = min(t);
    settling = t(settling_i);
    % min_v = min(d);
    % average_v = mean(d);
    % max_v = max(d);
    % steady_avr = mean(floor(length(d) * 0.9):end, d);
end

function [avr, vmin, vmax] = cal_statistic(t, d)
    tPulse = 1:0.1:t(end);
    avr = mean(tsa(d, t, tPulse));
    vmin = min(d);
    vmax = max(d);
end
