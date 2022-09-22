function patch_obj = draw_3kg_swarm_animation(patch_obj, P, R, ci, thrust)    
    GRID_SIZE = 0.5;%0.5*sqrt(2)/2;
    dP = [-1 1 0 -1 1 -1  1 -2  0  2;
        3  3 2  1 1 -1 -1 -2 -2 -2;
        0  0 0  0 0  0  0  0  0.5  0] * GRID_SIZE;


    R = squeeze(R);
    PP = R * dP + P';

    for i = 1:length(dP)
        draw_agent_quad_animation(patch_obj(i), PP(:, i)', R, ci);
        %quiver3(PP(1, i), PP(2, i), PP(3, i), thrust(1), thrust(2), thrust(3), 'magenta')
    end
end