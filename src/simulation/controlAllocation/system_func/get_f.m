%% ===================================
%
% calculate forces output vector
% eta: y-axis angles, n x 1 vector
% xi: x-axis angles, n x 1 vector
% F: force magnitude, n x 1 vector
%
%% ===================================
function Fi = get_f(xi, eta, F)

    if length(eta) == 1
        Fi = [(sin(xi) .* F);
            (-sin(eta) .* cos(xi) .* F);
            (cos(eta) .* cos(xi) .* F)];
    else
        Fi = [(sin(xi') .* F');
            (-sin(eta') .* cos(xi') .* F');
            (cos(eta') .* cos(xi') .* F')];
        n = length(eta);
        Fi = reshape(Fi, [3 * n 1]);
    end

end
