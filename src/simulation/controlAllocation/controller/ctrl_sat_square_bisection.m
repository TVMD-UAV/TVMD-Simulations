function [theta] = ctrl_sat_square_bisection(n, conf, b1r, b3r, f_r)
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