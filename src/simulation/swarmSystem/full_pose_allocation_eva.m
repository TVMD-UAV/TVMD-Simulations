addpath('../model/model')
addpath('../helper_functions')
[key, conf] = get_swarm_params();

b1r = [1; 0; 0];
b3r = [0; 0; 1];
b3r = b3r / norm(b3r);

f_r = [10; -20; -20];
% [theta, b3] = analytic(conf, b1r, b3r, f_r)
[theta, b3] = bisection(20, conf, b1r, b3r, f_r)

k = cross(b3r, f_r) / norm(cross(b3r, f_r));
nb3 = rot_vec_by_theta(b3r, k, theta);
nb2 = cross(nb3, b1r) / norm(cross(nb3, b1r));
nb1 = cross(nb2, nb3);
% nb1 = cross(nb2, nb3) / norm(cross(nb2, nb3));
Rd = [nb1 nb2 nb3];


close all
figure(1)
quiver3(0, 0, 0, f_r(1), f_r(2), f_r(3), 'red', 'DisplayName', '$f_r$'); hold on
quiver3(0, 0, 0, 5*nb3(1), 5*nb3(2), 5*nb3(3), 'Color', '#EDB120', 'DisplayName', "$b_{3}'$"); hold on
quiver3(0, 0, 0, 5*b3r(1), 5*b3r(2), 5*b3r(3), 'Color', '#7E2F8E', 'DisplayName', '$b_{3r}$'); hold on

quiver3(0, 0, 0, Rd(1, 1), Rd(2, 1), Rd(3, 1), 'red', 'LineWidth', 2, 'DisplayName', "$\mathbf{b}_1'$"); hold on
quiver3(0, 0, 0, Rd(1, 2), Rd(2, 2), Rd(3, 2), 'green', 'LineWidth', 2, 'DisplayName', "$\mathbf{b}_2'$"); hold on
quiver3(0, 0, 0, Rd(1, 3), Rd(2, 3), Rd(3, 3), 'blue', 'LineWidth', 2, 'DisplayName', "$\mathbf{b}_3'$"); hold on
legend('Interpreter', 'latex')
plot_est_boundary(conf, Rd)

axis equal
xlabel('x')
ylabel('y')
zlabel('z')

figure(2)
f_r2 = Rd' * f_r;
quiver3(0, 0, 0, f_r2(1), f_r2(2), f_r2(3), 'red', 'DisplayName', "$f_r'$"); hold on
plot_est_boundary(conf, eye(3))
axis equal
xlabel('x')
ylabel('y')
zlabel('z')

function [theta, b3] = bisection(n, conf, b1r, b3r, f_r)
    sigma_a = conf('sigma_a');
    b2r = cross(b3r, b1r);
    theta_max = pi;
    theta = theta_max / 2;
    if abs(f_r' * b2r) > f_r' * b3r * tan(sigma_a)
        k = cross(b3r, f_r) / norm(cross(b3r, f_r));
        for i=1:n
            b3 = rot_vec_by_theta(b3r, k, theta);
            b2 = cross(b3, b1r) / norm(cross(b3, b1r));
            d = abs(f_r' * b2) - f_r' * b3 * tan(sigma_a);
            if d < 0
                theta = theta - 0.5 * theta_max / (2^i);
            else
                theta = theta + 0.5 * theta_max / (2^i);
            end
        end
    else
        theta = 0;
    end
end

function out = rot_vec_by_theta(vec, rot_axis, theta)
    out = vec * cos(theta) + cross(rot_axis, vec) * sin(theta) + rot_axis * (rot_axis' * vec) * (1 - cos(theta));
end

function [theta, b3] = analytic(conf, b1r, b3r, f_r)
    sigma_a = conf('sigma_a');
    b2r = cross(b3r, b1r);
    if abs(f_r' * b2r) > f_r' * b3r * tan(sigma_a)
    % if f_r' * b3r < norm(f_r) * cos(sigma_a)
        k = cross(b3r, f_r) / norm(cross(b3r, f_r));
        w = f_r' * (-skew(b1r) - tan(sigma_a) * eye(3));
        A = w * b3r;
        B = -w * cross(k, b3r);
        % A = f_r' * b3r;
        % B = f_r' * cross(k, b3r);
        % C = norm(f_r) * cos(sigma_a);
        % theta = asin(C / sqrt(A^2 + B^2)) - atan2(A, B);
        theta = atan2(A, B);
        theta
    else
        theta = 0;
    end
    k = cross(b3r, f_r) / norm(cross(b3r, f_r));
    b3 = rot_vec_by_theta(b3r, k, theta);
    disp('ddd')
    cross(b1r, cross(k, b3r))
end

function plot_est_boundary(conf, R)
    psi = conf('psi');
    g = conf('g');
    sigma_a = conf('sigma_a');
    f_max = conf('f_max') / g;
    n = length(psi);

    n_x = sum(psi == 0);
    n_y = n - n_x;
    x_basis = @(z) f_max * n_x + (n_y ~= 0) * z * tan(sigma_a);
    y_basis = @(z) f_max * n_y + (n_x ~= 0) * z * tan(sigma_a);
    vertices = zeros([8 3]);
    i_x = [1 -1 -1 1];
    i_y = [1 1 -1 -1];
    vertices(:, 1) = [i_x * x_basis(0) i_x  * x_basis(f_max * n/2)];
    vertices(:, 2) = [i_y * y_basis(0) i_y  * y_basis(f_max * n/2)];
    vertices(:, 3) = [zeros([1 4]) ones([1 4]) * f_max * n/2];
    vertices = (R * (vertices'))';
    
    face = ones(4,1) * [0 4 5 1] + (1:4)';
    face(4, 3) = 5;
    face(4, 4) = 1;
    p = patch('Vertices',vertices,'Faces',face, 'EdgeColor','red','FaceColor','none','LineWidth',1);
    p.FaceAlpha = 0.3;
end