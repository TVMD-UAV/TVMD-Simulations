close all;
rng('default')
addpath('algorithms')
addpath('../model/model')
addpath('../model/model/swarm_conf')

function simplified_discrete_swarm_model(params, dt)
    pos = params('pos');
    psi = params('psi');
    n = length(psi);

    persistent sys_act
    persistent sys_prop
    if length(sys_act) <= 0
        sys_act = calc_actuator_discrete_model(params, dt);
    end
    if length(sys_prop) <= 0
        sys_prop = calc_propeller_discrete_model(params, dt);
    end

    for i = 1:n
    end
end

function sysd = calc_actuator_discrete_model(params, dt)
    mKp = params('mKp'); % Servo motor gain
    mKd = params('mKd'); % Servo motor gain

    A = [0 0 1 0;
         0 0 0 1;
         -mKp 0 -mKd 0;
         0 -mKp 0 -mKd];
    B = [0 0; 0 0; mKp 0; 0 mKp];
    C = [1 0 0 0; 0 1 0 0];

    motor_sys = sys(A, B, C, 0);
    sysd = c2d(motor_sys, dt);
end

function sysd = calc_propeller_discrete_model(params, dt)
    pKp = params('pKp'); % Servo motor gain
    A = -pKd;
    B = pKp;
    C = 1;

    prop_sys = sys(A, B, C, 0);
    sysd = c2d(prop_sys, dt);
end