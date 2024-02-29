function [R_d, theta, nb3] = calc_R_d(drone_params, ctrl_params, f_r, R_r, t)
    b1r = R_r(:, 1);
    b2r = R_r(:, 2);
    b3r = R_r(:, 3);
    [theta] = ctrl_sat_square_bisection(drone_params, ctrl_params, 30, b1r, b3r, f_r, t);
    if abs(theta) < 0.01
        % If f_r is almost parallel to b3r, then the rotation axis k will vanish
        theta = 0;
        nb3 = b3r;
        R_d = R_r;
    else
        k = cross(b3r, f_r) / norm(cross(b3r, f_r));
        nb3 = rot_vec_by_theta(b3r, k, theta);
        nb2 = cross(nb3, b1r) / norm(cross(nb3, b1r));
        nb1 = cross(nb2, nb3);
        R_d = [nb1 nb2 nb3];
    end
end
