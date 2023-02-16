function u_sat = force_sat(conf, u)
    psi = conf('psi');
    sigma_a = conf('sigma_a');
    f_max = conf('f_max');
    n = length(psi);
    n_x = sum(psi == 0);
    n_y = n - n_x;

    a = n_x * f_max + (n_y ~= 0) * u(3) * tan(sigma_a);
    b = n_y * f_max + (n_x ~= 0) * u(3) * tan(sigma_a);
    t = (a * b) / sqrt(u(1)^2 * b^2 + u(2)^2 * a^2);

    u_sat = u;
    if t < 1
        u_sat(1:2) = t * u(1:2);
    end
end