function eulers = rot2zxy_crossover(R)
    %global psi0 phi0 theta0;

    %phi = asin(R(:, 3, 2)); 
    psi = atan2(-R(:, 1, 2), R(:, 2, 2));    
    phi = atan2(R(:, 3, 2), sqrt(abs(1-R(:, 3, 2).^2)));
    theta = atan2(-R(:, 3, 1), R(:, 3, 3));

    psit = psi;
    phit = phi;
    thetat = theta;

    psi_offset = 0;
    phi_offset = 0;
    theta_offset = 0;
    
    for i=2:length(R)
        phi_offset = phi_offset + euler_crossover(phi(i), phi(i-1));
        phit(i) = phi_offset + phi(i);
        
        psi_offset = psi_offset + euler_crossover(psi(i), psi(i-1));
        psit(i) = psi_offset + psi(i);

        theta_offset = theta_offset + euler_crossover(theta(i), theta(i-1));
        thetat(i) = theta_offset + thetat(i);
    end

    % Update base
    eulers = [psit, phit, thetat];
end