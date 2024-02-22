function [a, b, F] = output_saturation(conf, n, a, b, F, a0, b0, tf0, dt)
    sigma_a = conf('sigma_a');
    sigma_b = conf('sigma_b');
    f_max = conf('f_max');

    r_sigma_a = conf('r_sigma_a');
    r_sigma_b = conf('r_sigma_b');
    r_f = conf('r_f');

    a_m_max = min(sigma_a, a0 + r_sigma_a * dt);
    a_m_min = max(-sigma_a, a0 - r_sigma_a * dt);
    b_m_max = min(sigma_b, b0 + r_sigma_b * dt);
    b_m_min = max(-sigma_b, b0 - r_sigma_b * dt);
    tf_m_max = min(f_max, tf0 + r_f * dt);
    tf_m_min = max(0, tf0 - r_f * dt);

    a = min(a_m_max, max(a_m_min, a));
    b = min(b_m_max, max(b_m_min, b));
    F = min(tf_m_max, max(tf_m_min, F));
end
