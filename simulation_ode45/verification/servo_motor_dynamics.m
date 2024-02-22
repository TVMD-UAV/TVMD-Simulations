close all
Kp = 10;
Kd = 1;

A = [0 1;
    -Kp -Kd];
B = [0; Kp];
C = [1 0];
D = 0;

sys = ss(A, B, C, D);
%[u,t] = gensig("square", 1,2);
[u, t] = gensig("sine", 1, 4);
%t = 0:0.01:4;
%u = t>1;
figure
lsim(sys, u, t)
grid on

Ts = 0.01;
[u, t] = gensig("sine", 1, 4, Ts);
sysd = c2d(sys, Ts)
lsim(sysd, u, t)

A = [0 0 1 0;
    0 0 0 1;
    -Kp 0 -Kd 0;
    0 -Kp 0 -Kd];
B = [0; 0; Kp; Kp];
C = eye(4);

sys = ss(A, B, C, D);
[u, t] = gensig("square", 1, 2);
%[u,t] = gensig("sine",1,4,0.05);

figure
lsim(sys, u, t)
grid on

figure
[u, t] = gensig("square", 1, 2, Ts);
sysd = c2d(sys, Ts);
lsim(sysd, u, t)

Ad = eye(4) + A * Ts;
Bd = B * Ts;
