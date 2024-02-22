function e = thrust_efficiency(eta, xi, F)
    vec = zeros(3, 1);

    for i = 1:length(F)
        R = Rx(eta(i)) * Ry(xi(i));
        vec = vec + R * [0; 0; F(i)];
    end

    e = 100 * norm(vec) / sum(F);
end
