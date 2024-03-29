%% ===================================
%
% calculate angles and forces from force vectors
% Fi: forces vector, 3 x n vector
%
%% ===================================
function [F, a, b] = inverse_input(Fi)
    n = length(Fi') / 3;
    Fi = reshape(Fi, [3, n]);
    F = sqrt(sum(Fi.^2, 1))';
    a = asin(-Fi(2, :)' ./ F);
    b = atan2(Fi(1, :)' ./ cos(a), Fi(3, :)' ./ cos(a));
end
