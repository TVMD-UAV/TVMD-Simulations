function z_dc = compensator(n, z, z_d, dt, mKp, mKd, sigma_x, sigma_y)
    w_n = sqrt(mKp);
    zeta = mKd / (2 * w_n);
    k = mKp;

    w_d = w_n * sqrt(1 - zeta^2);
    sigma = -zeta * w_n;

    C1 = (w_n / w_d) * exp(sigma * dt) * sin(w_d * dt + atan(w_d / -sigma));
    C2 = (exp(sigma * dt) / w_d) * sin(w_d * dt);
    C3 = (k / w_d) * (w_d + exp(sigma * dt) * (sigma*sin(w_d*dt) - w_d*cos(w_d*dt))) / w_n^2;

    z_dc = z_d;
    %return
    
    for i=1:n
        zi = z(6*(i-1)+1 : 6*i);
        x_zi = [1 0 0 0 0 0; 0 0 1 0 0 0] * zi;
        v_zi = [0 1 0 0 0 0; 0 0 0 1 0 0] * zi;

        z_di = z_d(4*(i-1)+1 : 4*(i-1)+2);
        delta_z_di = z_di - x_zi;

        M = (delta_z_di + (1-C3-C1)*x_zi - C2*v_zi) ./ (C3 * delta_z_di);
        z_dcr = x_zi + delta_z_di .* M;
        
        z_dcr(1) = min(sigma_x, max(-sigma_x, z_dcr(1)));
        z_dcr(2) = min(sigma_y, max(-sigma_y, z_dcr(2)));

        z_dc(4*(i-1)+1 : 4*(i-1)+2) = z_dcr;
    end
end
