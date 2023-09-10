% conf_name = "model_A4_inc";
conf_name = "model_A8_inc";

run('initialization/init_params_team.m')   
close all 

n_sample = 100;
t = linspace(0, 2 * pi, n_sample);
dt = t(2) - t(1);
% [options, ctrl_params, u_d] = CA_seq_without_pesudo_boundary1(env_params, drone_params, ctrl_params, t);
% [options, ctrl_params, u_d] = CA_seq_with_pesudo_boundary1(env_params, drone_params, ctrl_params, t);
% [options, ctrl_params, u_d] = CA_seq_with_post_enhance1(env_params, drone_params, ctrl_params, t);

[options, ctrl_params, u_d] = CA_seq_ebra(env_params, drone_params, ctrl_params, t);
% [options, ctrl_params, u_d] = CA_seq_cgi(env_params, drone_params, ctrl_params, t);
% [options, ctrl_params, u_d] = CA_seq_sqp(env_params, drone_params, ctrl_params, t);

n = length(drone_params.psi);
z0 = reshape(ones([1 length(drone_params.psi)]) .* [0 0 0 0 100 100]', [6*length(drone_params.psi) 1]);

vecs = zeros(6, n_sample);
metrics = zeros([5 n_sample]);
increment = zeros([2 n_sample]);

Tf = zeros([n n_sample]);
eta_x = zeros([n n_sample]);
eta_y = zeros([n n_sample]);

violation = zeros([3, n_sample]);

C_zo2z = kron(eye(length(drone_params.psi)), [1 0 0 0 0 0;0 0 1 0 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]);

for i = 1:n_sample
    % u_d(:, i) = force_sat(conf, u(:, i));
    % [eta, xi, F, v] = allocator_moore_penrose(u_d(:, i), conf);
    % [eta, xi, R, F] = allocator_interior_point(u_d(:, i), W, conf, a0, b0, f0);

    % [al0, bl0, fl0] = allocator_moore_penrose(u(:, i), conf);
    % [al0, bl0, fl0] = output_saturation2(conf, n, al0, bl0, fl0);
    % [eta, xi, F] = allocator_nullspace(u(:, i), conf, fl0, al0, bl0, dt);
    % [eta, xi, F] = allocator_nullspace(u(:, i), conf, f0, a0, b0, dt);
    
    % [eta, xi, F] = allocator_redistributed_nonlinear(u_d(:, i), conf, a0, b0, f0, W, dt);
    % [eta, xi, F] = allocator_null_redistr(u_d(:, i), conf, a0, b0, f0, W, dt);
    % [eta, xi, F] = allocator_null_redistr_moment_enhance(u_d(:, i), conf, a0, b0, f0, W, dt);
    % [eta, xi, F] = allocator_null_redistr_torque(u_d(:, i), conf, a0, b0, f0, W, dt);

    if ctrl_params.allocator_id == 1
        [Tf_d, eta_xd, eta_yd, N_esp, incre, bounds, raw, tw, intersections, validness, sat_order, remain_rank] = ...
            allocator_redistributed_nonlinear_moment_enhance(env_params, drone_params, ctrl_params, u_d(:, i), z0, dt, t);
    elseif ctrl_params.allocator_id == 2
        [Tf_d, eta_xd, eta_yd, N_esp, incre, bounds, raw] = allocator_cgi(env_params, drone_params, u_d(:, i), z0, dt, t);
    elseif ctrl_params.allocator_id == 3
        [Tf_d, eta_xd, eta_yd, bounds] = allocator_interior_point(env_params, drone_params, u_d(:, i), z0, dt, t);
    end

    [Tf0, eta_x0, eta_y0] = z2raw(n, z0, env_params);
    [eta_xd, eta_yd, Tf_d] = output_saturation(drone_params, eta_xd, eta_yd, Tf_d, eta_x0, eta_y0, Tf0, dt);
    vecs(:, i) = full_dof_mixing(drone_params.pos, drone_params.psi, eta_xd, eta_yd, Tf_d);

    %% evaluation
    [metrics(1, i), metrics(2, i), metrics(3, i), metrics(4, i)] = output_error(u_d(:, i), vecs(:, i));
    metrics(5, i) = thrust_efficiency(eta_xd, eta_yd, Tf_d, drone_params);

    % update z0
    z_d = calc_prop_raw_command(Tf_d, eta_xd, eta_yd, env_params, drone_params, ctrl_params);
    z0 = C_zo2z' * z_d;

    Tf(:, i) = Tf_d;    eta_x(:, i) = eta_xd;    eta_y(:, i) = eta_yd;

    lower_x = bounds(:, 1);
    upper_x = bounds(:, 2);
    lower_y = bounds(:, 3);
    upper_y = bounds(:, 4);

    mask_a0 = (eta_xd < lower_x) | (eta_xd > upper_x);
    mask_b0 = (eta_yd < lower_y) | (eta_yd > upper_y);
    mask_tf0 = (Tf_d > drone_params.f_max) | (Tf_d < 0);
    % mask_tf0
    violation(1, i) = sum(mask_a0);
    violation(2, i) = sum(mask_b0);
    violation(3, i) = sum(mask_tf0);
    increment(:, i) = incre;
end

matfilename = strcat(options('foldername'), options('filename'));
save(matfilename, 'drone_params', 'env_params', 'ctrl_params', ...
    't', 'u_d', 'vecs', 'eta_x', 'eta_y', ...
    'metrics', 'violation', 'incre');

plot_wrench(t, vecs, u_d, [], options);
plot_metrics(t, metrics, increment, options);
% plot_metrics(t, te, ef, em, df, dm, options);
violation

function [options, ctrl_params, u_d] = CA_seq_without_pesudo_boundary1(env_params, drone_params, ctrl_params, t)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_seq_cmd";
    filename = "without_pseudo_boundary";
    ctrl_params.pseudo_boundary = false;
    ctrl_params.post_torque_enhance = false;
    ctrl_params.allocator_id = 1;
    options = gen_project_options_subtask(projectpath, projectname, filename, t(end), true);
    u_d = get_ref_wrench2(env_params, drone_params, t);
end

function [options, ctrl_params, u_d] = CA_seq_with_pesudo_boundary1(env_params, drone_params, ctrl_params, t)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_seq_cmd";
    filename = "with_pseudo_boundary";
    ctrl_params.pseudo_boundary = true;
    ctrl_params.post_torque_enhance = false;
    ctrl_params.allocator_id = 1;
    options = gen_project_options_subtask(projectpath, projectname, filename, t(end), true);
    u_d = get_ref_wrench2(env_params, drone_params, t);
end

function [options, ctrl_params, u_d] = CA_seq_with_post_enhance1(env_params, drone_params, ctrl_params, t)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_seq_cmd";
    filename = "with_post_enhance";
    ctrl_params.pseudo_boundary = true;
    ctrl_params.post_torque_enhance = true;
    ctrl_params.allocator_id = 1;
    options = gen_project_options_subtask(projectpath, projectname, filename, t(end), true);
    u_d = get_ref_wrench2(env_params, drone_params, t);
end

function [options, ctrl_params, u_d] = CA_seq_ebra(env_params, drone_params, ctrl_params, t)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_seq_cmd_8A";
    filename = "ebra_test";
    ctrl_params.pseudo_boundary = true;
    ctrl_params.post_torque_enhance = true;
    ctrl_params.allocator_id = 1;
    options = gen_project_options_subtask(projectpath, projectname, filename, t(end), true);
    u_d = get_ref_wrench2(env_params, drone_params, t);
end

function [options, ctrl_params, u_d] = CA_seq_cgi(env_params, drone_params, ctrl_params, t)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_seq_cmd_8A";
    filename = "cgi_test";
    ctrl_params.allocator_id = 2;
    options = gen_project_options_subtask(projectpath, projectname, filename, t(end), true);
    u_d = get_ref_wrench2(env_params, drone_params, t);
end

function [options, ctrl_params, u_d] = CA_seq_sqp(env_params, drone_params, ctrl_params, t)
    projectpath = "C:\\Users\\NTU\\Documents\\Projects\\Multidrone\\outputs\\CA_evaluation";
    projectname = "CA_seq_cmd_8A";
    filename = "sqp_test";
    ctrl_params.allocator_id = 3;
    options = gen_project_options_subtask(projectpath, projectname, filename, t(end), true);
    u_d = get_ref_wrench2(env_params, drone_params, t);
end

function u = get_ref_wrench(env_params, drone_params, t)
    g = env_params.g;
    m = drone_params.m;
    f_amp = 1*m*g;
    m_amp = 0.5*m*g;
    k = 3;
    l = 0.2;
    u = [2*m*g + f_amp * sin(l * t.^k); f_amp * sin(l * t.^k + pi / 3); 0 * sin(l * t.^k + 2 * pi / 3) + 4*m*g;
        m_amp * sin(l * t.^k); m_amp * sin(l * t.^k + pi / 3); m_amp * sin(l * t.^k + 2 * pi / 3)];
end

function u = get_ref_wrench2(env_params, drone_params, t)
    g = env_params.g;
    m = drone_params.m;
    n = length(drone_params.psi);
    f_amp = 2*m*g;
    m_amp = 1*m*g;
    k = 3;
    l = 0.2;
    u = [2*m*g + f_amp * sin(l * t.^k); f_amp * sin(l * t.^k + pi / 3); 0 * sin(l * t.^k + 2 * pi / 3) + n*m*g;
        m_amp * sin(l * t.^k); m_amp * sin(l * t.^k + pi / 3); m_amp * sin(l * t.^k + 2 * pi / 3)];
end

function z_d = calc_prop_raw_command(Tf_d, eta_xd, eta_yd, env_params, drone_params, ctrl_params)
    % Parameters
    rho = env_params.rho; % kg/m3
    prop_d = env_params.prop_d; % 8 inch = 20.3 cm

    CT_u = env_params.CT_u; % upper propeller thrust coefficient
    CT_l = env_params.CT_l; % lower propeller thrust coefficient
    CP_u = env_params.CP_u; % upper propeller drag coefficient
    CP_l = env_params.CP_l; % lower propeller drag coefficient

    sigma_x = drone_params.sigma_a;
    sigma_y = drone_params.sigma_b;
    sigma_w0 = drone_params.sigma_w0;
    sigma_w = drone_params.prop_max;

    psi = drone_params.psi;
    n = length(psi);

    % Inversing
    beta_allo = [CT_u CT_l; prop_d * CP_u -prop_d * CP_l];

    z_d = zeros([4*n 1]);
    for i=1:n
        wm_d = sqrt(beta_allo \ [Tf_d(i); 0] / (rho * prop_d^4));
        z_di = [eta_xd(i); eta_yd(i); wm_d];

        if (ctrl_params.zd_pypass)
            z_d(4*(i-1)+1 : 4*i) = z_di;
        else
            z_low = [-sigma_x; -sigma_y;   sigma_w0;    sigma_w0];
            z_upp = [ sigma_x;  sigma_y; sigma_w(1);  sigma_w(2)];
            z_d(4*(i-1)+1 : 4*i) = min(z_upp, max(z_low, z_di));
        end
    end
end

function [eta_xd, eta_yd, Tf_d] = output_saturation(drone_params, eta_xd, eta_yd, Tf_d, eta_x0, eta_y0, Tf0, dt)
    n = length(drone_params.psi);
    % sigma_a = drone_params.sigma_a;
    % sigma_b = drone_params.sigma_b;
    % f_max = drone_params.f_max;

    % r_sigma_a = drone_params.r_sigma_a;
    % r_sigma_b = drone_params.r_sigma_b;
    % r_f = drone_params.r_f;

    % a_m_max = min(sigma_a, a0 + r_sigma_a * dt);
    % a_m_min = max(-sigma_a, a0 - r_sigma_a * dt);
    % b_m_max = min(sigma_b, b0 + r_sigma_b * dt);
    % b_m_min = max(-sigma_b, b0 - r_sigma_b * dt);
    % tf_m_max = min(f_max, tf0 + r_f * dt);
    % tf_m_min = max(0, tf0 - r_f * dt);

    % a = min(a_m_max, max(a_m_min, a));
    % b = min(b_m_max, max(b_m_min, b));
    % F = min(tf_m_max, max(tf_m_min, F));

    [lower_x, upper_x, lower_y, upper_y] = solve_upper_lower_bounds(drone_params, dt, Tf0, eta_x0, eta_y0);
    eta_xd = sat(eta_xd, lower_x, upper_x);
    eta_yd = sat(eta_yd, lower_y, upper_y);
    Tf_d = sat(Tf_d, 0, drone_params.f_max);
end
