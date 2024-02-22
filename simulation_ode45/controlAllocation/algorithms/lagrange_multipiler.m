addpath('../../helper_functions')
addpath('../system_func')

% syms eta xi tf
eta = sym('eta', 'real');
xi = sym('xi', 'real');
tf = sym('tf', 'real');
%syms m1 m2 m3 m4 m5 m6
m = sym('m%d', [6 1], 'real');

M = [mm(1) mm(4) mm(5);
    mm(4) mm(2) mm(6);
    mm(5) mm(6) mm(3)];
Fi = get_f(eta, xi, tf);

dFi = jacobian(Fi, [eta xi tf]);

(Fi' * M)'

% ans = simplify((Fi' * M * dFi)')
% latex(ans)
