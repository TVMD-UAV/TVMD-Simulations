function draw_agent_quad_animation(patch_obj, beam_obj, P, R, ci)
    R = squeeze(R);
    radius = 0.5 * sqrt(2) / 2;
    height = 0.2;
    k=1:8;
    kk = k*pi/2;
    vertices = zeros(8, 3);
    vertices(:, 1) = radius*cos(kk);
    vertices(:, 2) = radius*sin(kk);
    vertices(1:4, 3) = height;
    vertices(5:8, 3) = -height;
    vertices = P + vertices * R';

    mid_beam = [0 -radius 0; 0 radius 0];
    mid_beam = P + mid_beam * R';

    % Colormap
    cmap = parula(256);
    uint8(ci);
    c = cmap(uint8(ci), :);

    k=1:4;
    face = ones(4,1) * [0 4 5 1] + k';
    face(4, 3) = 5;
    face(4, 4) = 1;
    patch_obj.Vertices = vertices;
    patch_obj.Faces = face;
    patch_obj.EdgeColor = c;

    beam_obj.XData = mid_beam(:, 1);
    beam_obj.YData = mid_beam(:, 2);
    beam_obj.ZData = mid_beam(:, 3);
    alpha(0.3);
end