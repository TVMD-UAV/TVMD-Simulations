close all;
rng('default')
addpath('../helper_functions')

eta = 0.5;
xi = 0.5;

R = Rx(eta) * Ry(xi);
P = [1 1 -1 -1 0;
    1 -1 1 -1 0;
    0 0 0 0 0];
F = [1 1 1 1 1]';

method1(R, P, F)
method2(R, P, F)

function vec = method1(R, P, F)
    vec = zeros(6, 1);

    for i = 1:length(F)
        vec = vec + ...
            [R * [0; 0; 1] * F(i);
            cross(P(:, i), R * [0; 0; 1] * F(i))];
    end

end

function vec = method2(R, P, F)
    RR = [R zeros(3);
        zeros(3) R];
    vec = zeros(6, 1);

    for i = 1:length(F)
        vec = vec + ...
            [[0; 0; 1];
            -cross([0; 0; 1], R' * P(:, i))] * F(i);
    end

    vec = RR * vec;
end

function [eta, xi, R, F] = control_allocation(u, P)
    eta = atan2(-u(2), u(3));
    xi = atan2(cos(eta) * u(1), u(3));
    R = Rx(eta) * Ry(xi);
    bar_P = R' * P;
    bar_u = [R zeros(3);
        zeros(3) R]' * u;

    B = skew([0; 0; -1]) * bar_P;
    B = [ones(1, length(P));
        B(1:2, :)];
    %F = (B' * B) \ B' * bar_u(3:5);
    %F = B' / (B * B') * bar_u(3:5);
    F = lsqlin(B, bar_u(3:5), -eye(10), zeros(10, 1));
end
