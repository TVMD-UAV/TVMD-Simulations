syms t
syms theta
syms vec [3 1] matrix
syms R [3 3] matrix
syms dR [3 3] matrix
syms omega [3 3] matrix

syms x_r [3 1] matrix
syms v_r [3 1] matrix
syms a_r [3 1] matrix

syms h_tilt
syms T_tilt
syms h
syms T

assume(t, "positive")
assume(h_tilt, "real")
assume(T_tilt, "real")
assume(h, "real")
assume(T, "real")

% Translation
x_r = [0; 0; (0.5 * h * (1 - cos(pi * t / T)))];
v_r = diff(x_r, t);
a_r = diff(v_r, t);

fprintf("\n\nx_r = \n")
pretty(x_r)
fprintf("\n\nv_r = \n")
pretty(v_r)
fprintf("\n\na_r = \n")
pretty(a_r)


T = double(subs(T, 2));
h = double(subs(h, 1.5));
t_s = linspace(0, T, 100);
x_s = double(subs(x_r, {'t', 'T', 'h'}, {t_s, T, h}));
v_s = double(subs(v_r, {'t', 'T', 'h'}, {t_s, T, h}));
a_s = double(subs(a_r, {'t', 'T', 'h'}, {t_s, T, h}));

figure 
subplot(3, 1, 1)
plot(t_s, x_s); hold on
subplot(3, 1, 2)
plot(t_s, v_s); hold on
subplot(3, 1, 3)
plot(t_s, a_s); hold on
% return

% Rotational
% self.manual_sp.y = 1000 * h_tilt * (1 + math.cos(2 * math.pi / T_tilt * (t - t_m)))
theta = subs(theta, h_tilt * (1 + cos(2 * pi / T_tilt * t)));
vec = subs(vec, [1; 0; 0]);

q = [cos(theta/2); sin(theta/2)*vec]; % q0, q1, q2, q3
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