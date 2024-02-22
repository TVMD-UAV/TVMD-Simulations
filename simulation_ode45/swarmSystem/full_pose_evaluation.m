addpath('../model/model')
addpath('../controlAllocation/controller')
addpath('../controlAllocation/system_func')
[key, conf] = get_swarm_params("model_A9_con");

output_path = "H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\outputs\\230208_fullpose_evaluation\\";
foldername = "100cases\\";
output_path = output_path + foldername;

b1r = [1; 0; 0];
b3r = [0; 0; 1];
b3r = b3r / norm(b3r);
r_xy = 0.5;


t = linspace(-pi/2, 3*pi/2, 100);
% t = linspace(0.4, 1.2, 10);

f_r = [0*t; cos(t); sin(t)];
theta_bis = zeros([length(t) 1]);
for i=1:length(t)
    theta_bis(i) = ctrl_sat_square_bisection(20, conf, b1r, b3r, f_r(:,i));
    % pause
end
figure('Position', [10 210 500 400])
plot(t-pi/2, theta_bis,'DisplayName','Bisection','LineWidth',2, 'Color', '#0072BD', 'Marker', 'o', 'MarkerSize', 3); hold on
legend('Interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10)
title("Results with Different $\mathbf{f}_r$", 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 18)
xlabel('$t$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
ylabel('$\theta$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
% saveas(gcf, output_path + 'full_pose_eva_distribution.svg');
% saveas(gcf, output_path + 'full_pose_eva_distribution.fig');
% saveas(gcf, output_path + 'full_pose_eva_distribution.eps');

% return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = linspace(-3*pi/2, pi/2, 10);
f_r = [0*t; cos(t); sin(t)];
theta_ana = zeros([length(t) 1]);
theta_bis = zeros([length(t) 1]);
for i=1:length(t)
    [theta_bis(i), theta_ana(i)] = test(b3r, f_r(:, i), r_xy);
end

% close all
figure('Position', [10 210 500 400])
plot(t, theta_bis,'DisplayName','Bisection','LineWidth',2, 'Color', '#0072BD', 'Marker', 'o', 'MarkerSize', 3); hold on
plot(t, theta_ana,'DisplayName','Analytic','LineWidth',2, 'Color', '#D95319', 'Marker', 'x', 'MarkerSize', 3); hold on
legend('Interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10)
title("Results with Different $\mathbf{f}_r$", 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 18)
xlabel('$t$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
ylabel('$\theta$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
% saveas(gcf, output_path + 'full_pose_eva_distribution.svg');
% saveas(gcf, output_path + 'full_pose_eva_distribution.fig');
% saveas(gcf, output_path + 'full_pose_eva_distribution.eps');

figure('Position', [510 210 1000 400])
cmap = jet(length(t)); 
subplot(1, 2, 1)
for i=1:length(t)
    quiver(0, 0, f_r(2, i), f_r(3, i), 'Color', cmap(i, :)); hold on 
    k = cross(b3r, f_r(:, i)) / norm(cross(b3r, f_r(:, i)));
    b3 = rot_vec_by_theta(b3r, k, theta_bis(i));
    quiver(0, 0, b3(2), b3(3), 'Color', cmap(i, :), 'LineStyle', '--', 'LineWidth', 2); hold on 
end
title("Bisection Results", 'FontName', 'Times New Roman', 'FontSize', 18)
xlabel('y', 'FontName', 'Times New Roman', 'FontSize', 12)
ylabel('z', 'FontName', 'Times New Roman', 'FontSize', 12)

subplot(1, 2, 2)
for i=1:length(t)
    quiver(0, 0, f_r(2, i), f_r(3, i), 'Color', cmap(i, :)); hold on 
    k = cross(b3r, f_r(:, i)) / norm(cross(b3r, f_r(:, i)));
    b3 = rot_vec_by_theta(b3r, k, theta_ana(i));
    quiver(0, 0, b3(2), b3(3), 'Color', cmap(i, :), 'LineStyle', '--', 'LineWidth', 2); hold on 
end
title("Analytic Solutions", 'FontName', 'Times New Roman', 'FontSize', 18)
xlabel('y', 'FontName', 'Times New Roman', 'FontSize', 12)
ylabel('z', 'FontName', 'Times New Roman', 'FontSize', 12)
% saveas(gcf, output_path + 'full_pose_eva_sol_compare.svg');
% saveas(gcf, output_path + 'full_pose_eva_sol_compare.fig');
% saveas(gcf, output_path + 'full_pose_eva_sol_compare.eps');

function [theta_bis, theta_ana] = test(b3r, f_r, r_xy)
    n = 20;
    theta_bis = bisection(b3r, f_r, r_xy, n);
    theta_ana = analytic(b3r, f_r, r_xy);
    % fprintf("bisection: %.4f \nanalytic:  %.4f\n", theta_bis * 180 / pi, theta_ana * 180 / pi);
end

function [theta, b3] = bisection(b3r, f_r, r_xy, n)
    k = cross(b3r, f_r) / norm(cross(b3r, f_r));
    % theta_max = asin(norm(k));
    theta_max = asin(norm(cross(b3r, f_r))/norm(f_r));
    % theta_max = pi;
    theta = theta_max / 2;
    for i=1:n
        b3 = rot_vec_by_theta(b3r, k, theta);
        if f_r' * b3 >= sqrt(f_r' * f_r - r_xy^2)
            theta = theta - 0.5 * theta_max / (2^i);
        else
            theta = theta + 0.5 * theta_max / (2^i);
        end
    end
end

function [theta, b3] = analytic(b3r, f_r, r_xy)
    if f_r' * b3r < sqrt(norm(f_r)^2 - r_xy^2)
        k = cross(b3r, f_r) / norm(cross(b3r, f_r));
        A = f_r' * b3r;
        B = f_r' * cross(k, b3r);
        % theta_max = asin(norm(k));
        % B = norm(f_r) * sin(theta_max);
        % B = sqrt(norm(f_r)^2 - norm(f_r' * b3r)^2);
        C = sqrt(norm(f_r)^2 - r_xy^2);
        theta = asin(C / sqrt(A^2 + B^2)) - atan2(A, B);
    else
        theta = 0;
    end
    k = cross(b3r, f_r) / norm(cross(b3r, f_r));
    b3 = rot_vec_by_theta(b3r, k, theta);
end

function out = rot_vec_by_theta(vec, rot_axis, theta)
    out = vec * cos(theta) + cross(rot_axis, vec) * sin(theta) + rot_axis * (rot_axis' * vec) * (1 - cos(theta));
end