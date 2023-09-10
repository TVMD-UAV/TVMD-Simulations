conf_name = "model_A4_inc";
% conf_name = "model_A8_inc";

run('initialization/init_params_team.m')   
close all 


n = length(drone_params.psi);
z0 = reshape(ones([1 length(drone_params.psi)]) .* [0 0 0 0 100 100]', [6*length(drone_params.psi) 1]);

% Single command evaluation
% [options, ctrl_params, u_d] = CA_single_without_pesudo_boundary1(ctrl_params);
% [options, ctrl_params, u_d] = CA_single_with_pesudo_boundary1(ctrl_params);
% [options, ctrl_params, u_d] = CA_single_with_pose_enhance1(ctrl_params);

% [options, ctrl_params, u_d, z0] = CA_single_PBP_without_pesudo_boundary2(ctrl_params, drone_params, z0);
% [options, ctrl_params, u_d, z0] = CA_single_PBP_with_pesudo_boundary2(ctrl_params, drone_params, z0);
[options, ctrl_params, u_d, z0] = CA_single_A4_Inc_vis(ctrl_params, drone_params, z0);

% [options, ctrl_params, u_d, z0] = CA_single_without_pesudo_boundary2(ctrl_params, drone_params, z0);
% [options, ctrl_params, u_d, z0] = CA_single_with_pesudo_boundary2(ctrl_params, drone_params, z0);
% [options, ctrl_params, u_d, z0] = CA_single_with_pose_enhance2(ctrl_params, drone_params, z0);

% z0 = reshape(ones([1 length(drone_params.psi)]) .* [0 0 0 0 200 200]', [6*length(drone_params.psi) 1]);
% u_d = [10 0 30 0 0 3]';

[vec, eta_x0, eta_y0, Tf_0] = actuator_mixing(z0, drone_params, env_params);
u0 = full_dof_mixing(drone_params.pos, drone_params.psi, eta_x0, eta_y0, Tf_0);

[Tf_d, eta_xd, eta_yd, N_esp, incre, bounds, raw, tw, intersections, validness, sat_order, remain_rank] = ...
    allocator_redistributed_nonlinear_moment_enhance(env_params, drone_params, ctrl_params, u_d, z0, dt);
% [Tf_d, eta_xd, eta_yd, N_esp, incre, bounds, raw] = allocator_cgi(env_params, drone_params, u_d, z0, dt, 0);

cmdsd = [eta_xd eta_yd Tf_d];
zo = kron(eye(length(drone_params.psi)), [1 0 0 0 0 0;0 0 1 0 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]) * z0;
[vec, eta_x0, eta_y0, Tf_0] = actuator_mixing(zo, drone_params, env_params);
cmds0 = [eta_x0 eta_y0 Tf_0];

u = full_dof_mixing(drone_params.pos, drone_params.psi, eta_xd, eta_yd, Tf_d);
[ef, em, df, dm] = output_error(u_d, u);
te = thrust_efficiency(eta_xd, eta_yd, Tf_d, drone_params);
% 
% Output results
fprintf("\n\n\n Input and Output wrench\n");
fprintf("& $\\mathbf{u}_0$ ");      fprintf("& %.3f ", u0);      fprintf("\\\\ \n");
fprintf("& $\\mathbf{u}_d$ ");    fprintf("& %.3f ", u_d);    fprintf("\\\\ \\hline\n");

fprintf("& $\\mathbf{u}$ ");      fprintf("& %.3f ", u);      fprintf("\\\\ \n");
fprintf("\\cmidrule(lr){2-8}\n");
fprintf("& Error ");    fprintf("& \\multicolumn{3}{c}{%.3f} & \\multicolumn{3}{c}{%.3f} ", ef, em);  fprintf("\\\\  \n");
fprintf("& Alignment ");    fprintf("& \\multicolumn{3}{c}{%.3f} & \\multicolumn{3}{c}{%.3f} ", df, dm);  fprintf("\\\\  \n");
fprintf("& Increment ");    fprintf("& \\multicolumn{3}{c}{%.3f} & \\multicolumn{3}{c}{%.3f} ", incre(1), incre(2));  fprintf("\\\\  \n");
% fprintf("%.3f & %.3f & %.3f & %.3f & %.3f \\\\\n", ef, em, df, dm, te);


% Output bounds
fprintf("\n\n\n Bounds and allocation results\n");
fprintf("Agent ID & $\\sigma_{ix-}^\\star$ & $\\eta_{ix0}$ & $\\eta_{ixd}$ & $\\sigma_{ix+}^\\star$ & \n $\\sigma_{iy-}^\\star$ & $\\eta_{iy0}$ & $\\eta_{iyd}$ & $\\sigma_{iy+}^\\star$ & \n $T_{f, \\min}$ & $T_{f0}$ & $T_{fd}$ & $T_{f,\\max}$ \\\\ \\midrule\n");
for i=1:n
    fprintf("& %d & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f & %.2f \\\\ \n", ...
            i, bounds(i, 1), eta_x0(i), eta_xd(i), bounds(i, 2), ...
            bounds(i, 3), eta_y0(i), eta_yd(i), bounds(i, 4), ...
            0, Tf_0(i), Tf_d(i), drone_params.f_max);
end


% Output solutions
% fprintf("\n Intersection solutions\n");
% fprintf("$k$ ");
% for i=1:n
%     fprintf("& $t_{%d,x}$ & $t_{%d,y}$ & $t_{%d,z}$ ", i, i, i);
% end
% fprintf("\\\\  \\midrule\n");
% for i=1:n % iteration
%     fprintf("& %d ", i);
%     for j=1:n % agent
%         fprintf("& %.2f & %.2f & %.2f ", tw(i, j, 1), tw(i, j, 2), tw(i, j, 3));
%     end
%     fprintf("\\\\\n");
% end

% Solution at each iteration
fprintf("\n\n\n Solution in Iteration\n");
fprintf("Agent ID & $\\sigma_{ix-}^\\star$ & $\\eta_{ix0}$ & $\\eta_{ixd}$ & $\\sigma_{ix+}^\\star$ & \n $\\sigma_{iy-}^\\star$ & $\\eta_{iy0}$ & $\\eta_{iyd}$ & $\\sigma_{iy+}^\\star$ & \n $T_{f, \\min}$ & $T_{f0}$ & $T_{fd}$ & $T_{f,\\max}$ \\\\ \\midrule\n");
for i=1:n+1
    fprintf("%d", i);
    fprintf(", %.2f", raw(i, :));
    fprintf("\n");
end

% Summary
fprintf("\n Summary\n");
fprintf("Final increment: %.4f\n", incre);
fprintf("Rank remains: %df\n", remain_rank);
fprintf("Sat order: "); fprintf("%d ", sat_order); fprintf("\n"); 

% plot_allocation_result(env_params, drone_params, n, bounds, cmds0, cmdsd, raw, tw, intersections, options);
plot_team_allocation_3d(drone_params, bounds, eta_xd, eta_yd, Tf_d, 0.04, options);

figure('Position', [10 10 800 800])
scaling = 0.01;
f_xyz = reshape(get_f(eta_x0, eta_y0, Tf_0), [3 n]) * 0;
plot_3kg_swarm(drone_params, [0 0 0], eye(3), 1, scaling * f_xyz); hold on
% plot_est_boundary_elliptic_cone(drone_params, eye(3), [0 0 0], scaling);
plot_attainable_force_space_monte(drone_params, 1000000, scaling);
quiver3(0, 0, 0, scaling*vec(1), scaling*vec(2), scaling*vec(3), 'LineStyle', '-', 'Color', '#FF0000', 'LineWidth', 2, 'AutoScale', 'off'); hold on 
set(gcf, 'Renderer', 'painters')
set(gca, 'DataAspectRatio', [1 1 1])
grid on

campos([-3 3 4]);
xlabel('x', 'FontName', 'Times New Roman', 'FontSize', 12)
ylabel('y', 'FontName', 'Times New Roman', 'FontSize', 12)
zlabel('z', 'FontName', 'Times New Roman', 'FontSize', 12)
set(gcf, 'PaperPosition', [0 0 20 20])
set(gcf, 'PaperSize', [20 20])
options('savepdf') = true;
savefig_helper(options, '_attainable_3d');

function [options, ctrl_params, u_d] = CA_single_without_pesudo_boundary1(ctrl_params)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_single_cmd";
    filename = "without_pseudo_boundary";
    ctrl_params.pseudo_boundary = false;
    ctrl_params.post_torque_enhance = false;
    u_d = [10 0 30 0 0 3]';
    options = gen_project_options_subtask(projectpath, projectname, filename, 1, true);
end

function [options, ctrl_params, u_d] = CA_single_with_pesudo_boundary1(ctrl_params)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_single_cmd";
    filename = "with_pseudo_boundary";
    ctrl_params.pseudo_boundary = true;
    ctrl_params.post_torque_enhance = false;
    u_d = [10 0 30 0 0 3]';
    options = gen_project_options_subtask(projectpath, projectname, filename, 1, true);
end

function [options, ctrl_params, u_d] = CA_single_with_pose_enhance1(ctrl_params)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_single_cmd";
    filename = "with_post_enhance";
    ctrl_params.pseudo_boundary = true;
    ctrl_params.post_torque_enhance = true;
    u_d = [10 0 30 0 0 3]';
    options = gen_project_options_subtask(projectpath, projectname, filename, 1, true);
end


function [options, ctrl_params, u_d, z0] = CA_single_PBP_without_pesudo_boundary2(ctrl_params, drone_params, z0)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_single_cmd";
    filename = "without_pseudo_boundary2";
    ctrl_params.pseudo_boundary = false;
    ctrl_params.post_torque_enhance = false;
    u_d = [10 0 40 0 5 5]';
    z0 = reshape(ones([1 length(drone_params.psi)]) .* [0 0 0 0 150 150]', [6*length(drone_params.psi) 1]);

    z0(1) = drone_params.sigma_a;
    z0(5) = drone_params.prop_max(1);
    z0(6) = drone_params.prop_max(1);
    z0(12+1) = drone_params.sigma_a;
    z0(12+5) = drone_params.prop_max(1);
    z0(12+6) = drone_params.prop_max(1);
    options = gen_project_options_subtask(projectpath, projectname, filename, 1, false);
end

function [options, ctrl_params, u_d, z0] = CA_single_PBP_with_pesudo_boundary2(ctrl_params, drone_params, z0)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_single_cmd";
    filename = "with_pseudo_boundary2";
    ctrl_params.pseudo_boundary = true;
    ctrl_params.post_torque_enhance = false;
    u_d = [10 0 40 0 5 5]';
    z0 = reshape(ones([1 length(drone_params.psi)]) .* [0 0 0 0 150 150]', [6*length(drone_params.psi) 1]);
    
    z0(1) = drone_params.sigma_a;
    z0(5) = drone_params.prop_max(1);
    z0(6) = drone_params.prop_max(1);
    z0(12+1) = drone_params.sigma_a;
    z0(12+5) = drone_params.prop_max(1);
    z0(12+6) = drone_params.prop_max(1);
    options = gen_project_options_subtask(projectpath, projectname, filename, 1, false);
end

function [options, ctrl_params, u_d, z0] = CA_single_with_pesudo_boundary2(ctrl_params, drone_params, z0)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_single_cmd";
    filename = "with_pseudo_boundary2";
    ctrl_params.pseudo_boundary = true;
    ctrl_params.post_torque_enhance = false;
    % u_d = [10 0 30 5 0 3]';
    u_d = [20 0 60 10 0 6]';
    z0(1) = drone_params.sigma_a;
    options = gen_project_options_subtask(projectpath, projectname, filename, 1, true);
end

function [options, ctrl_params, u_d, z0] = CA_single_with_pose_enhance2(ctrl_params, drone_params, z0)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_single_cmd";
    filename = "with_post_enhance2";
    ctrl_params.pseudo_boundary = true;
    ctrl_params.post_torque_enhance = true;
    % u_d = [10 0 30 5 0 3]';
    u_d = [20 0 60 10 0 6]';
    z0(1) = drone_params.sigma_a;
    options = gen_project_options_subtask(projectpath, projectname, filename, 1, true);
end

function [options, ctrl_params, u_d, z0] = CA_single_A4_Inc_vis(ctrl_params, drone_params, z0)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "A4_inc_viz";
    filename = "A4_inc_viz";
    ctrl_params.pseudo_boundary = true;
    ctrl_params.post_torque_enhance = true;
    u_d = [10 0 40 0 3 3]';
    z0 = reshape(ones([1 length(drone_params.psi)]) .* [0 0 0 0 200 200]', [6*length(drone_params.psi) 1]);
    
    z0(1) = drone_params.sigma_a;
    z0(5) = drone_params.prop_max(1);
    z0(6) = drone_params.prop_max(1);
    z0(12+1) = drone_params.sigma_a;
    z0(12+5) = drone_params.prop_max(1);
    z0(12+6) = drone_params.prop_max(1);
    options = gen_project_options_subtask(projectpath, projectname, filename, 1, true);
end

function plot_allocation_result(env_params, drone_params, n, bounds, cmds0, cmdsd, raw, tw, intersections, options)
    f_max = drone_params.f_max;

    % region [boundary visualization]
    figsize = 400;
    num_row = 2;
    figure('Position', [10 10 figsize*ceil(n/num_row) figsize*num_row])
    set(gcf, 'Renderer', 'painters')
    cmap = turbo(n); 

    for i = 1:n
        subplot(2, ceil(n / 2), i);
        plot_admissible_with_bounds(bounds(i, :), f_max);
        title('Agent ' + string(i), 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 14);
        xlabel('$$\mathbf{b}_x$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
        ylabel('$$\mathbf{b}_y$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
        zlabel('$$\mathbf{b}_z$$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
        axis equal

        eta_x0 = cmds0(i, 1);  eta_y0 = cmds0(i, 2);  Tf_0 = cmds0(i, 3);
        u = get_f(eta_x0, eta_y0, Tf_0);

        eta_xd = cmdsd(i, 1);  eta_yd = cmdsd(i, 2);  Tf_d = cmdsd(i, 3);
        u_d = get_f(eta_xd, eta_yd, Tf_d);

        
        lower_x = bounds(i, 1); upper_x = bounds(i, 2); 
        lower_y = bounds(i, 3); upper_y = bounds(i, 4);

        f0 = raw(1, 3*i-2 : 3*i);
        f0_feasibility = [(lower_x <= eta_x0) & (eta_x0 <= upper_x) (lower_y <= eta_y0) & (eta_y0 <= upper_y) (0 <= Tf_0) & (Tf_0 <= f_max)];
        fd_feasibility = [(lower_x <= eta_xd) & (eta_xd <= upper_x) (lower_y <= eta_yd) & (eta_yd <= upper_y) (0 <= Tf_d) & (Tf_d <= f_max)];
        fprintf("Agent %d\n", i)
        fprintf("f0: %d%d%d | fd: %d%d%d | raw f0: %.4f, %.4f, %.4f \n", f0_feasibility, fd_feasibility, f0);

        quiver3(0, 0, 0, u(1), u(2), u(3), 'Color', '#000000', 'LineWidth', 1, 'LineStyle', '--', 'AutoScale', 'off'); hold on
        quiver3(0, 0, 0, u_d(1), u_d(2), u_d(3), 'Color', '#0000AA', 'LineWidth', 2, 'LineStyle', '--', 'AutoScale', 'off'); hold on

        % for each allocation
        for j=1:n
            if sum(raw(j+1, :)) == 0  % End of allocation
                break;
            end
            f0 = raw(j, 3*i-2 : 3*i)';
            f = raw(j+1, 3*i-2 : 3*i)';
            ff = squeeze(intersections(j, i, 1, :));
            fn = squeeze(intersections(j, i, 2, :));
            fp = squeeze(intersections(j, i, 3, :));
            df = f - f0;
            if norm(df) < 1e-10
                % scatter3(f0(1), f0(2), f0(3), 'Color', '#AA0000', 'LineWidth', 2, 'Marker', '*'); hold on
            else 
                quiver3(f0(1), f0(2), f0(3), df(1), df(2), df(3), 'Color', cmap(j, :), 'LineWidth', 2, 'AutoScale', 'off'); hold on
                % scatter3(ff(1), ff(2), ff(3), 'Color', cmap(j, :), 'LineWidth', 2, 'Marker', 'x'); hold on
                plot3(ff(1), ff(2), ff(3), 'Color', cmap(j, :), 'LineWidth', 2, 'Marker', 'o'); hold on
                plot3([f(1) ff(1)], [f(2) ff(2)], [f(3) ff(3)], 'Color', cmap(j, :), 'LineWidth', 1, 'LineStyle', '--'); hold on

                % plot3(fn(1), fn(2), fn(3), 'Color', cmap(j, :), 'LineWidth', 2, 'Marker', 'x'); hold on
                % plot3([f(1) fn(1)], [f(2) fn(2)], [f(3) fn(3)], 'Color', cmap(j, :), 'LineWidth', 1, 'LineStyle', ':'); hold on
                % plot3(fp(1), fp(2), fp(3), 'Color', cmap(j, :), 'LineWidth', 2, 'Marker', '+'); hold on
                % plot3([f(1) fp(1)], [f(2) fp(2)], [f(3) fp(3)], 'Color', cmap(j, :), 'LineWidth', 1, 'LineStyle', ':'); hold on
                
                % for k=1:3
                %     ff = f0 + tw(j, i, k) .* (f-f0);
                %     scatter3(ff(1), ff(2), ff(3), 'Color', cmap(j, :));
                % end
            end
            [Tf_s, eta_xs, eta_ys] = inverse_input(n, raw(j+1, :)');
            fs_feasibility = [(lower_x <= eta_xs(i)) & (eta_xs(i) <= upper_x) (lower_y <= eta_ys(i)) & (eta_ys(i) <= upper_y) (0 <= Tf_s(i)) & (Tf_s(i) <= f_max)];
            fprintf("%d%d%d|", fs_feasibility);
            % pause
        end
        fprintf("\nTw:\n");

        for j=1:n
            fprintf("<%d> \t%.4f \t%.4f \t%.4f\n", j, squeeze(tw(j, i, :)))
        end
    end
    % endregion [boundary visualization]
    % colorbar 
    % sat_order
    set(gcf, 'PaperPosition', [0 0 20 20])
    set(gcf, 'PaperSize', [20 20])
    options('savepdf') = true;
    savefig_helper(options, '_agent_allocation');
end


function plot_team_allocation_3d(drone_params, bounds, eta_xd, eta_yd, Tf_d, scale, options)
    n = length(drone_params.psi);
    figure('Position', [10 10 800 800])
    camproj perspective
    set(gca, 'DataAspectRatio', [1 1 1])    
    set(gcf, 'Renderer', 'painters')
    f0 = get_f(eta_xd, eta_yd, Tf_d);
    f0 = reshape(f0, [3 n]) * scale;
    plot_3kg_swarm(drone_params, [0 0 0], eye(3), 1, f0); 
    for i = 1:n
        p = drone_params.pos(:, i);
        R = Rz(drone_params.psi(i));
        text(p(1), p(2), p(3), strcat("     Agent", num2str(i)));
        lower_x = -drone_params.sigma_a;
        upper_x = drone_params.sigma_a; 
        lower_y = -drone_params.sigma_b; 
        upper_y = drone_params.sigma_b;
        plot_admissible_force_space(drone_params, [lower_x upper_x lower_y upper_y], R, p, scale, 0.2);
        % plot_admissible_force_space(drone_params, bounds(i, :), R, p, scale, 0.5);
        % plot_est_boundary_elliptic_cone(drone_params, R, p, 0.1)
        colors = ["r", "g", "b"];
        texts = ["x", "y", "z"];
        for j=1:3
            basis = zeros([3 1]);
            basis(j) = scale;
            b = R * basis;
            quiver3(p(1), p(2), p(3), b(1), b(2), b(3), colors(j), "LineWidth", 2);
            text(p(1)+2*b(1), p(2)+2*b(2), p(3)+2*b(3), texts(j));
        end
    end
    grid on
    campos([-3 3 4]);
    xlabel('x', 'FontName', 'Times New Roman', 'FontSize', 12)
    ylabel('y', 'FontName', 'Times New Roman', 'FontSize', 12)
    zlabel('z', 'FontName', 'Times New Roman', 'FontSize', 12)

    % set(gcf, 'PaperUnit', 'normalized')
    % set(gcf, 'PaperPosition', [0 0 1 1])
    set(gcf, 'PaperPosition', [0 0 20 20])
    set(gcf, 'PaperSize', [20 20])
    options('savepdf') = true;
    savefig_helper(options, '_allocation_3d');
end