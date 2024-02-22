function patch_obj = draw_3kg_swarm_animation(drone_params, patch_obj, quiver_obj, beam_obj, P, R, ci, f0)    
    dP = drone_params.pos;
    psi = drone_params.psi;

    R = squeeze(R);
    PP = R * dP + P';

    for i = 1:length(dP)
        thrust = R * Rz(psi(i)) * f0(:, i);
        draw_agent_quad_animation(patch_obj(i), beam_obj(i), PP(:, i)', R * Rz(psi(i)), ci);
        quiver_obj(i).XData = PP(1, i);
        quiver_obj(i).YData = PP(2, i);
        quiver_obj(i).ZData = PP(3, i);
        quiver_obj(i).UData = thrust(1);
        quiver_obj(i).VData = thrust(2);
        quiver_obj(i).WData = thrust(3);
        % quiver3(PP(1, i), PP(2, i), PP(3, i), thrust(1), thrust(2), thrust(3), 'magenta')
    end
end