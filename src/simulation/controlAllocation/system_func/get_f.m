%% ===================================
%
% calculate forces output vector
% eta: y-axis angles, n x 1 vector
% xi: x-axis angles, n x 1 vector
% F: force magnitude, n x 1 vector
%
%% ===================================
function Fi = get_f(eta, xi, F)
    Fi = [(sin(xi) .* F);
        (-sin(eta) .* cos(xi) .* F);
        (cos(eta) .* cos(xi) .* F); ];
end
