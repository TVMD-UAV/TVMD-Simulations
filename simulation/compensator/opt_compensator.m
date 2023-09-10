function z_dc = opt_compensator(n, z, z_d, sigma_x, sigma_y)
    z_dc = z_d;
    for i=1:n
        zi = z(6*(i-1)+1 : 6*i);
        x_zi = [1 0 0 0 0 0; 0 0 1 0 0 0] * zi;
        v_zi = [0 1 0 0 0 0; 0 0 0 1 0 0] * zi;

        z_di = z_d(4*(i-1)+1 : 4*(i-1)+2);
        z_dcr = -14.3636 * (x_zi-z_di) - 0.3070 * (v_zi-0) + 0.6924 * z_di; % Tracking
        % z_dcr = -28.0209 * (x_zi-z_di) -0.5379 * (v_zi-0) + 0.9971 * z_di; % Team evaluation
        % z_dcr = -17.3915 * (x_zi-z_di) -0.4492 * (v_zi-0) + 0.8054 * z_di;
        % z_dcr = -11.8288 * (x_zi-z_di) -0.4958 * (v_zi-0) + 0.4905 * z_di;
        
        z_dcr(1) = min(sigma_x, max(-sigma_x, z_dcr(1)));
        z_dcr(2) = min(sigma_y, max(-sigma_y, z_dcr(2)));

        z_dc(4*(i-1)+1 : 4*(i-1)+2) = z_dcr;
    end
end
