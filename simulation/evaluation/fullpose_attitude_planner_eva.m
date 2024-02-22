close all
conf_name = "model_A4_inc";
% conf_name = "model_A4_con";
run('initialization/init_params_team.m')    


b1r = [1; 0; 0];
b3r = [0; 0; 1];
b3r = b3r / norm(b3r);
r_xy = 0.5;


% t = linspace(-pi/2, 3*pi/2, 100);
% t = linspace(-3*pi/2, pi/2, 100);
t = linspace(-pi, pi, 100);
% t = linspace(0.4, 1.2, 10);

% f_r = [0*t; sin(t); cos(t)];
f_r = [sin(t); 0*t; cos(t)];
theta_bis = zeros([length(t) 1]);
for i=1:length(t)
    theta_bis(i) = ctrl_sat_square_bisection(drone_params, 20, b1r, b3r, f_r(:,i), 0);
    % pause
end
figure('Position', [10 210 400 350])
% plot(t-pi/2, theta_bis,'DisplayName','Bisection','LineWidth',2, 'Color', '#0072BD', 'Marker', 'none', 'MarkerSize', 3); hold on
plot(t, theta_bis,'DisplayName','Bisection','LineWidth',2, 'Color', '#0072BD', 'Marker', 'none', 'MarkerSize', 3); hold on
% legend('Interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 10)
title("$\theta$ in Different $\mathbf{f}_r$", 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 18)
xlabel('$t$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)
ylabel('$\theta$', 'interpreter', 'latex', 'FontName', 'Times New Roman', 'FontSize', 12)


R = zeros([3 3 length(t)]);
figure('Position', [510 210 400 350])
cmap = jet(length(t)); 
% subplot(1, 2, 1)
for i=1:length(t)
    quiver(0, 0, f_r(2, i), f_r(3, i), 'Color', cmap(i, :)); hold on 
    k = cross(b3r, f_r(:, i)) / norm(cross(b3r, f_r(:, i)));
    b3 = rot_vec_by_theta(b3r, k, theta_bis(i));
    b2 = cross(b3, b1r) / norm(cross(b3, b1r));
    b1 = cross(b2, b3);
    quiver(0, 0, b3(2), b3(3), 'Color', cmap(i, :), 'LineStyle', '--', 'LineWidth', 2); hold on 

    R(:, :, i) = [b1 b2 b3];
end

% generate answers for PX4 testing in pfa_att_planner_test.cpp
% print_px4_attitude_test_data("expected_replanned", t, R, 20)
num_test = 20;
fprintf("const int num_test = %d;\n", num_test);
print_px4_attitude_test_data("theta_ans", t, theta_bis, num_test)
print_px4_attitude_test_data("f_r_ans", t, f_r, num_test)
print_px4_attitude_test_data("R_ans", t, R, num_test)

title("Bisection Results", 'FontName', 'Times New Roman', 'FontSize', 18)
xlabel('y', 'FontName', 'Times New Roman', 'FontSize', 12)
ylabel('z', 'FontName', 'Times New Roman', 'FontSize', 12)
axis equal

% b1r = [1; 0; 0];
% b3r = [0; 0; 1];
% b3r = b3r / norm(b3r);
f_r = [10; -20; -20];
% f_r = [0; 0; -4];

[theta] = ctrl_sat_square_bisection(drone_params, 20, b1r, b3r, f_r, 0);

k = cross(b3r, f_r) / norm(cross(b3r, f_r));
nb3 = rot_vec_by_theta(b3r, k, theta);
nb2 = cross(nb3, b1r) / norm(cross(nb3, b1r));
nb1 = cross(nb2, nb3);
% nb1 = cross(nb2, nb3) / norm(cross(nb2, nb3));
Rd = [nb1 nb2 nb3];

figure(3)
scaling = 0.1;
f_r = scaling * f_r;
quiver3(0, 0, 0, f_r(1), f_r(2), f_r(3), 'red', 'DisplayName', '$f_r$', 'LineWidth', 2); hold on
% quiver3(0, 0, 0, 5*nb3(1), 5*nb3(2), 5*nb3(3), 'Color', '#EDB120', 'DisplayName', "$b_{3}'$", 'LineWidth', 2); hold on
quiver3(0, 0, 0, b3r(1), b3r(2), b3r(3), 'Color', '#7E2F8E', 'DisplayName', '$\mathbf{b}_{3r}$', 'LineWidth', 2); hold on

quiver3(0, 0, 0, Rd(1, 1), Rd(2, 1), Rd(3, 1), 'red', 'LineWidth', 2, 'DisplayName', "$\mathbf{b}_{1d}$"); hold on
quiver3(0, 0, 0, Rd(1, 2), Rd(2, 2), Rd(3, 2), 'green', 'LineWidth', 2, 'DisplayName', "$\mathbf{b}_{2d}$"); hold on
quiver3(0, 0, 0, Rd(1, 3), Rd(2, 3), Rd(3, 3), 'blue', 'LineWidth', 2, 'DisplayName', "$\mathbf{b}_{3d}$"); hold on
legend('Interpreter', 'latex')
% plot_est_boundary(conf, Rd)
plot_est_boundary_elliptic_cone(drone_params, Rd, [0;0;0], scaling)

axis equal
xlabel('x')
ylabel('y')
zlabel('z')
return

% figure(2)
% f_r2 = Rd' * f_r;
% quiver3(0, 0, 0, f_r2(1), f_r2(2), f_r2(3), 'red', 'DisplayName', "$f_r'$"); hold on
% % plot_est_boundary(conf, eye(3))
% plot_est_boundary_elliptic_cone(conf)
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')

function print_px4_attitude_test_data(var_name, t, data, num_samples)
    depth = length(size(data));
    if size(data, 1) == 1 || size(data, 2) == 1
        depth = depth - 1;
    end
    interval = floor(length(t) / num_samples);
    type_prefix = "const float";

    fprintf("%s %s", type_prefix, var_name);
    if depth == 1
        fprintf("[num_test]");
    elseif depth == 2
        fprintf("[num_test][%d]", size(data, 1));
    elseif depth == 3
        fprintf("[num_test][%d][%d]", size(data, 1), size(data, 2));
    end

    fprintf(" = {");
    for i=1:interval:length(t)
        if depth == 1
            fprintf("%f", data(i))
        elseif depth == 2
            fprintf("{%f, %f, %f}", data(1, i), data(2, i), data(3, i))
        elseif depth == 3
            fprintf("{{%f, %f, %f}, {%f, %f, %f}, {%f, %f, %f}}", ...
                data(1, 1, i), data(2, 1, i), data(3, 1, i), ...
                data(1, 2, i), data(2, 2, i), data(3, 2, i), ...
                data(1, 3, i), data(2, 3, i), data(3, 3, i)) 
        end

        if i + interval < length(t)
            fprintf(", ");
        end
    end
    fprintf("}\n\n");
end