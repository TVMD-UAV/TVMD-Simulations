function [F_partial] = allocator_nullspace_cal_jacobian(conf)
    psi = conf('psi');

    n = length(psi);

    a_s = sym('a_%d', [n 1]);
    b_s = sym('b_%d', [n 1]);
    f_s = sym('f_%d', [n 1]);

    %% Linear approaximation derivation
    F = [];

    for i = 1:n
        F = [F; get_f(a_s(i), b_s(i), f_s(i))];
    end

    F_partial_a = jacobian(F, a_s);
    F_partial_b = jacobian(F, b_s);
    F_partial_f = jacobian(F, f_s);
    F_partial = [F_partial_a F_partial_b F_partial_f];

    F_partial = matlabFunction(F_partial, 'Vars', {a_s, b_s, f_s});
end
