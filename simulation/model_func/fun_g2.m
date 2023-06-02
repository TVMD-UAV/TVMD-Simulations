function gk = fun_g2(n_k, sigma_k_bar, z, f_max)
    if (sigma_k_bar >= pi / 2)
        gk = n_k * f_max;
    else
        gk = (n_k > 0) * z * tan(sigma_k_bar);
    end
end
