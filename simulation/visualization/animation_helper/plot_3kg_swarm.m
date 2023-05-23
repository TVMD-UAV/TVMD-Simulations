function [patch_obj, quiver_obj, beam_obj] = plot_3kg_swarm(drone_params, P, R, ci, f0)
    dP = drone_params.pos;
    psi = drone_params.psi;

    R = squeeze(R);
    PP = R * dP + P';

    patch_obj = gobjects([length(dP), 1]);
    beam_obj = gobjects([length(dP), 1]);
    quiver_obj = gobjects([length(dP), 1]);

    for i = 1:length(dP)
        thrust = R * Rz(psi(i)) * f0(:, i);
        [patch_obj(i), beam_obj(i)] = draw_agent_quad(PP(:, i)', R * Rz(psi(i)), ci);
        quiver_obj(i) = quiver3(PP(1, i), PP(2, i), PP(3, i), thrust(1), thrust(2), thrust(3), 'magenta'); hold on
    end

end
