% Get the static matrix determined by configuration
function M = get_M(n, psi, P)
    M = zeros(6, 3 * n);

    for i = 1:n
        M(1:3, (i - 1) * 3 + 1:i * 3) = Rz(psi(i));
        M(4:6, (i - 1) * 3 + 1:i * 3) = skew(P(:, i)) * Rz(psi(i));
    end

end
