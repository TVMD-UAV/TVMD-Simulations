function [pos, psi] = model_A4_inc(GRID_SIZE)
    pos = [ 1  0 0 -1;
            0 -1 1  0;
            0  0 0  0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [pi/2 0 0 pi/2];
end