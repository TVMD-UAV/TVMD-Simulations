%% ===================================
%
% calculate forces output vector
% a: x-axis angles, n x 1 vector
% b: y-axis angles, n x 1 vector
% f: force magnitude, n x 1 vector
%
%% ===================================
function Fi = get_f(a, b, tf)

    if length(a) == 1
        Fi = [(cos(a) .* sin(b).* tf);
            (-sin(a) .* tf);
            (cos(a) .* cos(b) .* tf)];
    else
        Fi = [(cos(a') .* sin(b').* tf');
            (-sin(a') .* tf');
            (cos(a') .* cos(b') .* tf')];
        n = length(a);
        Fi = reshape(Fi, [3 * n 1]);
    end

end
