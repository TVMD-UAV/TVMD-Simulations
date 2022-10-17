load('mine_norm_profile.mat')
mine_bnorm_att = bnorm_att;
mine_bnorm_ang = bnorm_ang;
mine_bnorm_pos = bnorm_pos;
mine_bnorm_vel = bnorm_vel;
mine_t = t;

load('birotor_norm_profile.mat')
birotor_bnorm_att = bnorm_att;
birotor_bnorm_ang = bnorm_ang;
birotor_bnorm_pos = bnorm_pos;
birotor_bnorm_vel = bnorm_vel;
birotor_t = t;

lineStyle = '--';
markerStyle = 'none';

projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\0805\\comparison_with_birotor\\';
foldername = 'norm_plot\\';
filename = 'birotor';

labely_pos = -0.8;

% Draw orientation
figure('Position', [10 10 540 400])
subplot(4, 1, 1);
plot(mine_t, mine_bnorm_att, 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
plot(birotor_t, birotor_bnorm_att, 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
labely = ylabel('$\Vert\tilde{q}\Vert$ (rad)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
labely.Position(1) = labely_pos;
%xlabel('Time (s)')
%ylim([-0.1 0.1])
%title('Norm of orientation error')
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
% Draw angular velocity
subplot(4, 1, 2);
plot(mine_t, mine_bnorm_ang, 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
plot(birotor_t, birotor_bnorm_ang, 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
%plot(t, vecnorm(W - beta, 2, 2), 'DisplayName', '$$\Vert\tilde{\Omega}\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
labely = ylabel('$\Vert\tilde{\Omega}\Vert$ (rad/s)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
labely.Position(1) = labely_pos;
%xlabel('Time (s)')
%ylim([-0.2 0.2])
%title('Norm of angular velocity error')
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
% Draw position
subplot(4, 1, 3);
plot(mine_t, mine_bnorm_pos, 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
plot(birotor_t, birotor_bnorm_pos, 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
%plot(t, vecnorm(traj(:, 1:3, 1) - P, 2, 2), 'DisplayName', '$$\Vert\tilde{p}\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
labley = ylabel('$\Vert\tilde{p}\Vert$ (m)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
labely.Position(1) = labely_pos;
pos = labely.Position
pos(1) = labely_pos
labley.Position = pos
%xlabel('Time (s)')
%title('Norm of position error')
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
% Draw velocity
subplot(4, 1, 4);
plot(mine_t, mine_bnorm_vel, 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
plot(birotor_t, birotor_bnorm_vel, 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#0072BD'); hold on
%plot(t, vecnorm(traj(:, 1:3, 2) - dP, 2, 2), 'DisplayName', '$$\Vert\tilde{v}\Vert$$', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
labely = ylabel('$\Vert\tilde{v}\Vert$ (m/s)', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
labely.Position(1) = labely_pos;
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12)
%ylim([-10 10])
%title('Norm of velocity error')
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
% Draw acceleration

saveas(gcf, strcat(projectpath, foldername, filename, '_norm.svg'));
saveas(gcf, strcat(projectpath, foldername, filename, '_norm.fig'));
