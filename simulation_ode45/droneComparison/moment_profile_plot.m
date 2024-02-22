projectpath = 'H:\\.shortcut-targets-by-id\\1_tImZc764OguGZ7irM7kqDx9_f6Tdqwi\\National Taiwan University\\Research\\Multidrone\\VTswarm\\src\\simulation\\model\\outputs\\0805\comparison_with_birotor\moment_plot\\';
foldername = 'test\\';
filename = 'moment_plot';

key = {'projectpath', 'foldername', 'filename'};
value = {projectpath, foldername, filename};
options = containers.Map(key, value);

load('mine_moment_profile.mat')
mine_B_M_f = B_M_f;
mine_B_M_d = B_M_d;
mine_B_M_g = B_M_g;
mine_B_M_a = B_M_a;
mine_t = t;

load('birotor_moment_profile.mat')
birotor_B_M_f = B_M_f;
birotor_B_M_d = B_M_d;
birotor_B_M_g = B_M_g;
birotor_B_M_a = B_M_a;
birotor_t = t;

%%%
labely_pos = -0.35;
tt_pos = 0.012;

figure('Position', [10 10 1680 500])
subplot(3, 4, 1)
yyaxis left
plot(mine_t, mine_B_M_f(:, 1), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
labely = ylabel('Nm', 'FontName', 'Times New Roman', 'FontSize', 12);
%labely.Position(1) = labely_pos;
ylim([-1e-4 1e-4])
yyaxis right
plot(birotor_t, birotor_B_M_f(:, 1), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on

ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;
xlim([0 4]);
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
title('$M_{\rm{T},x}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
%titlet = title('Thrust moment $M_f$', 'FontName', 'Times New Roman', 'FontSize', 12, 'Interpreter', 'latex');
%tpos = titlet.Position;
%tpos(2) = tt_pos;
%titlet.Position = tpos;

subplot(3, 4, 5)
yyaxis left
plot(mine_t, mine_B_M_f(:, 2), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
labely = ylabel('Nm', 'FontName', 'Times New Roman', 'FontSize', 12);
% labely.Position(1) = labely_pos;
yyaxis right
plot(birotor_t, birotor_B_M_f(:, 2), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;
xlim([0 4]);
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
title('$M_{\rm{T},y}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

subplot(3, 4, 9)
yyaxis left
plot(mine_t, mine_B_M_f(:, 3), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
labely = ylabel('Nm', 'FontName', 'Times New Roman', 'FontSize', 12);
% labely.Position(1) = labely_pos;
ylim([-1e-4 1e-4])
yyaxis right
plot(birotor_t, birotor_B_M_f(:, 3), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;

xlim([0 4]);
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12)
title('$M_{\rm{T},z}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

%%%
%labely_pos = -2;

subplot(3, 4, 1 + 1)
yyaxis left
plot(mine_t, mine_B_M_d(:, 1), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
ylim([-1e-5 1e-5])
yyaxis right
plot(birotor_t, birotor_B_M_d(:, 1), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;
ylim([-1e-2 1e-2])

xlim([0 4]);
%labely = ylabel('$M_{d,x}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
%labely.Position(1) = labely_pos;
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
title('$M_{\rm{D},x}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

%titlet = title('Drag moment $M_d$', 'FontName', 'Times New Roman', 'FontSize', 12, 'Interpreter', 'latex');
%tpos = titlet.Position;
%tpos(2) = tt_pos;
%titlet.Position = tpos;

subplot(3, 4, 5 + 1)
yyaxis left
plot(mine_t, mine_B_M_d(:, 2), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
ylim([-1e-5 1e-5])
yyaxis right
plot(birotor_t, birotor_B_M_d(:, 2), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;
ylim([-1e-2 1e-2])
%ylim([-1e-5 1e-5])

xlim([0 4]);
% labely = ylabel('$M_{d,y}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
% labely.Position(1) = labely_pos;
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
title('$M_{\rm{D},y}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

subplot(3, 4, 9 + 1)
yyaxis left
plot(mine_t, mine_B_M_d(:, 3), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
ylim([-1e-3 1e-3])
yyaxis right
plot(birotor_t, birotor_B_M_d(:, 3), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;
ylim([-1e-2 1e-2])

xlim([0 4]);
% labely = ylabel('$M_{d,z}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
% labely.Position(1) = labely_pos;
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12)
title('$M_{\rm{D},z}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

%%%
%labely_pos = -2;

subplot(3, 4, 1 + 2)
yyaxis left
plot(mine_t, mine_B_M_g(:, 1), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
ylim([-1e-5 1e-5])
yyaxis right
plot(birotor_t, birotor_B_M_g(:, 1), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;

xlim([0 4]);
% labely = ylabel('$M_{g,x}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
% labely.Position(1) = labely_pos;
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
title('$M_{\rm{G},x}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
%titlet = title('Gyroscopic moment $M_g$', 'FontName', 'Times New Roman', 'FontSize', 12, 'Interpreter', 'latex');
%titlet
%tpos = titlet.Position;
%tpos(2) = 0.0016;
%titlet.Position = tpos;

subplot(3, 4, 5 + 2)
yyaxis left
plot(mine_t, mine_B_M_g(:, 2), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
ylim([-1e-5 1e-5])
yyaxis right
plot(birotor_t, birotor_B_M_g(:, 2), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;
ylim([-2e-3 2e-3])
xlim([0 4]);
% labely = ylabel('$M_{g,y}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
% labely.Position(1) = labely_pos;
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
title('$M_{\rm{G},y}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

subplot(3, 4, 9 + 2)
yyaxis left
plot(mine_t, mine_B_M_g(:, 3), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
ylim([-1e-7 1e-7])
yyaxis right
plot(birotor_t, birotor_B_M_g(:, 3), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -7;
ax.YAxis(2).Exponent = -7;
ylim([-1e-7 1e-7])
xlim([0 4]);
% labely = ylabel('$M_{g,z}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12, 'Interpreter', 'latex');
% labely.Position(1) = labely_pos;
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12)
title('$M_{\rm{G},z}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

%%%
%labely_pos = -2;

subplot(3, 4, 1 + 3)
yyaxis left
plot(mine_t, mine_B_M_a(:, 1), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
ylim([-1e-3 1e-3])
yyaxis right
plot(birotor_t, birotor_B_M_a(:, 1), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;
ylim([-1e-3 1e-3])
xlim([0 4]);
% labely = ylabel('$M_{a,x}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
% labely.Position(1) = labely_pos;
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
title('$M_{\rm{A},x}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
% titlet = title('Adverse reactionary moment $M_a$', 'FontName', 'Times New Roman', 'FontSize', 12, 'Interpreter', 'latex');
% tpos = titlet.Position;
% tpos(2) = 0.000112;
% titlet.Position = tpos;

subplot(3, 4, 5 + 3)
yyaxis left
plot(mine_t, mine_B_M_a(:, 2), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
ylim([-1e-3 1e-3])
yyaxis right
plot(birotor_t, birotor_B_M_a(:, 2), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -3;
ax.YAxis(2).Exponent = -3;
ylim([-1e-3 1e-3])
xlim([0 4]);
% labely = ylabel('$M_{a,y}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
% labely.Position(1) = labely_pos;
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
title('$M_{\rm{A},y}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)

subplot(3, 4, 9 + 3)
yyaxis left
plot(mine_t, mine_B_M_a(:, 3), 'DisplayName', 'Proposed', 'LineWidth', 2, 'LineStyle', '-', 'Color', '#0072BD'); hold on
ylim([-1e-7 1e-7])
yyaxis right
plot(birotor_t, birotor_B_M_a(:, 3), 'DisplayName', 'Bi-rotor', 'LineWidth', 2, 'LineStyle', '--', 'Color', '#D95319'); hold on
ax = gca;
ax.YAxis(1).Exponent = -7;
ax.YAxis(2).Exponent = -7;
ylim([-1e-7 1e-7])
xlim([0 4]);
% labely = ylabel('$M_{a,z}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12);
% labely.Position(1) = labely_pos;
hl = legend('show', 'FontName', 'Times New Roman', 'FontSize', 10);
set(hl, 'Interpreter', 'latex')
xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12)
title('$M_{\rm{A},z}$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_state_norm.svg'));
saveas(gcf, strcat(options('projectpath'), options('foldername'), options('filename'), '_state_norm.fig'));
