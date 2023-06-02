function e = thrust_efficiency(eta_x, eta_y, tf, drone_params)
    % vec = zeros(3, 1);
    pos = drone_params.pos;
    psi = drone_params.psi;
    vec = full_dof_mixing(pos, psi, eta_x, eta_y, tf);

    e = 100 * norm(vec(1:3)) / sum(tf);
end
