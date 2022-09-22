function patch_obj = plot_3kg_swarm(P, R, ci, thrust)
    GRID_SIZE = 0.5; %0.5*sqrt(2)/2;
    dP = [-1 1 0 -1 1 -1 1 -2 0 2;
        3 3 2 1 1 -1 -1 -2 -2 -2;
        0 0 0 0 0 0 0 0 0.5 0] * GRID_SIZE;

    R = squeeze(R);
    PP = R * dP + P';

    patch_obj = gobjects([length(dP), 1]);

    for i = 1:length(dP)
        patch_obj(i) = draw_agent_quad(PP(:, i)', R, ci);
        %quiver3(PP(1, i), PP(2, i), PP(3, i), thrust(1), thrust(2), thrust(3), 'magenta')
    end

end
