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
    b = atan2(-Fi(2, :), Fi(3, :))';
    a = atan2(Fi(1, :), Fi(3, :) / cos(b'))';
end
