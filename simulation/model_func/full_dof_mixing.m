function vec = full_dof_mixing(P, psi, a, b, tf)
    n = length(psi);
    M = get_M(n, psi, P);
    fi = get_f(a, b, tf);
    vec = M * fi;
end 
