addpath('initialization')  
addpath('system_model')  
addpath('system_model/team_conf')  

conf_name = "model_A6_inc";
run('initialization/init_params_team_icra.m')    

P = drone_params.pos;
psi = drone_params.psi;

n = length(psi);
N_esp = ones([n 1]);
M = get_M(n, psi, P);


u = ones([6 1]);
N_esp(3) = 0;
N_esp(4) = 0;

% Original
f1 = cal_pinv(N_esp, M, u);
f2 = cal_geninv(N_esp, M, u);
[f1 f2]

function f = cal_pinv(N_esp, M, u)
    pinv_tol = 0.0001;

    M_esp = M * kron(diag(N_esp), eye(3)); % masking
    M_esp_dag = pinv(M_esp, pinv_tol);
    f = M_esp_dag * u;
end

function f = cal_geninv(N_esp, M, u)
    MWMT = M * M';
    M_esp = M * kron(diag(N_esp), eye(3)); % masking

    % removing column i
    for i = 1:length(N_esp)
        if N_esp(i) == 1
            continue;
        end
        % for j=0:2
        %     v = M(:, 3*(i-1)+j+1);
        %     MWMT = MWMT - v*v';
        % end
        v = M(:, 3*(i-1):3*(i-1)+2);
        MWMT = MWMT - v*v';
    end

    % Fast
    u_M = MWMT \ u;
    f = M_esp' * u_M;
end