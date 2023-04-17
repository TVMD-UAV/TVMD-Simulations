function [theta] = ctrl_sat_square_bisection(n, conf, b1r, b3r, f_r)
    sigma_a = conf('sigma_a');
    b2r = cross(b3r, b1r);
    theta_max = acos(b3r' * f_r / norm(f_r));
    theta = theta_max / 2;
    if ~is_attainable(conf, b1r, b2r, b3r, f_r);
        k = cross(b3r, f_r) / norm(cross(b3r, f_r));
        for i=1:n
            b3 = rot_vec_by_theta(b3r, k, theta);
            b2 = cross(b3, b1r) / norm(cross(b3, b1r));
            b1 = cross(b2, b3);
            if is_attainable(conf, b1, b2, b3, f_r)
                theta = theta - 0.5 * theta_max / (2^i);
            else
                theta = theta + 0.5 * theta_max / (2^i);
            end
            % fprintf("%d: %.4f\n", i, theta)
        end
    else
        theta = 0;
    end
end

function feasible = is_attainable(conf, b1, b2, b3, f_r)
    feasible = elliptic_cone(conf, b1, b2, b3, f_r);
    % feasible = pie(conf, b1, b2, b3, f_r);
end

function feasible = is_attainable_fat(conf, b1, b2, b3, f_r)
    sigma_a = conf('sigma_a');
    feasible = (abs(f_r' * b2) < f_r' * b3 * tan(sigma_a)) & ...
               (abs(f_r' * b1) < f_r' * b3 * tan(sigma_b));
end

function feasible = pie(conf, b1, b2, b3, f_r)
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');

    n = length(psi);
    n_x = sum(psi == 0);
    n_y = n - n_x;

    feasible = (n_x | abs(f_r' * b1) < f_r' * b3 * tan(sigma_b)) & ...
               (n_y | abs(f_r' * b2) < f_r' * b3 * tan(sigma_a));
end

function feasible = elliptic_cone(conf, b1, b2, b3, f_r)
    psi = conf('psi');
    n = length(psi);
    f_max = conf('f_max');
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');

    n_x = sum(psi == 0);
    n_y = n - n_x;

    z = f_r' * b3;
    % c_x = n_x * f_max;
    % c_y = z * tan(sigma_a);
    % c_x = fun_g(true, n_x, sigma_a, n_y, sigma_b, z, f_max) + fun_g(false, n_x, sigma_a, n_y, sigma_b, z, f_max);
    % c_y = fun_g(true, n_y, sigma_b, n_x, sigma_a, z, f_max) + fun_g(false, n_y, sigma_b, n_x, sigma_a, z, f_max);
    c_x = fun_g(n_x, sigma_b, z, f_max) + fun_g(n_y, sigma_a, z, f_max);
    c_y = fun_g(n_x, sigma_a, z, f_max) + fun_g(n_y, sigma_b, z, f_max);

    feasible = (z > 0) & (((f_r' * b1 / c_x)^2 + (f_r' * b2 / c_y)^2) <= 1);
end

% function gk = fun_g(principle_axis, n_kp, sigma_kp, n_k, sigma_k, z, f_max)
%     if (principle_axis || sigma_kp >= pi/2)
%         gk = n_kp * f_max;
%     else
%         gk = (n_k > 0) * abs(z) * tan(sigma_k);
%     end
% end

function gk = fun_g(n_k, sigma_k_bar, z, f_max)
    if (sigma_k_bar >= pi / 2)
        gk = n_k * f_max;
    else
        gk = (n_k > 0) * z * tan(sigma_k_bar);
    end
end