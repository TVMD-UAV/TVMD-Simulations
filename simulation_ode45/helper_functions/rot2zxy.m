function eulers = rot2zxy(R)
    %phi = asin(R(:, 3, 2)); 
    psi = atan2(-R(1, 2), R(2, 2));    
    phi = atan2(R(3, 2), sqrt(1-R(3, 2).^2));
    theta = atan2(-R(3, 1), R(3, 3));
    eulers = [psi, phi, theta];
end