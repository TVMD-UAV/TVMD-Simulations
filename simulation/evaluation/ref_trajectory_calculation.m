syms t
syms theta
syms v [3 1] matrix
syms R [3 3] matrix
syms dR [3 3] matrix
syms omega [3 3] matrix

syms h_tilt
syms T_tilt

assume(t, "positive")
assume(h_tilt, "real")
assume(T_tilt, "real")

% Rotational
% self.manual_sp.y = 1000 * h_tilt * (1 + math.cos(2 * math.pi / T_tilt * (t - t_m)))
theta = subs(theta, h_tilt * (1 + cos(2 * pi / T_tilt * t)));
v = subs(v, [1; 0; 0]);

q = [cos(theta/2); sin(theta/2)*v]; % q0, q1, q2, q3
R = [1-2*(q(3)^2+q(4)^2), 2*(q(2)*q(3)-q(1)*q(4)), 2*(q(2)*q(4)+q(1)*q(3));
     2*(q(2)*q(3)+q(1)*q(4)), 1-2*(q(2)^2+q(4)^2), 2*(q(3)*q(4)-q(1)*q(2));
     2*(q(2)*q(4)-q(1)*q(3)), 2*(q(3)*q(4)+q(1)*q(2)), 1-2*(q(2)^2+q(3)^2)];
dR = diff(R, t);

W_skew = R' * dR;
W = [W_skew(3,2); W_skew(1,3); W_skew(2,1)];

dW = diff(W, t);

fprintf("\n\nq = \n")
pretty(symmatrix2sym(q))
fprintf("\n\nR = \n")
pretty(symmatrix2sym(R))
fprintf("\n\nW = \n")
pretty(symmatrix2sym(W))
fprintf("\n\ndW = \n")
pretty(symmatrix2sym(dW))

T_tilt = double(subs(T_tilt, 2));
h_tilt = double(subs(h_tilt, pi/4));

t_s = linspace(0, T_tilt, 100);
q_s = double(subs(symmatrix2sym(q), {'t', 'T_tilt', 'h_tilt'}, {t_s, T_tilt, h_tilt}));
R_s = double(subs(symmatrix2sym(R), {'t', 'T_tilt', 'h_tilt'}, {t_s, T_tilt, h_tilt}));
W_s = double(subs(symmatrix2sym(W), {'t', 'T_tilt', 'h_tilt'}, {t_s, T_tilt, h_tilt}));
dW_s = double(subs(symmatrix2sym(dW), {'t', 'T_tilt', 'h_tilt'}, {t_s, T_tilt, h_tilt}));

figure 
subplot(3, 1, 1)
plot(t_s, q_s); hold on
subplot(3, 1, 2)
% plot(t_s, R_s); hold on
plot(t_s, W_s); hold on
subplot(3, 1, 3)
plot(t_s, dW_s); hold on