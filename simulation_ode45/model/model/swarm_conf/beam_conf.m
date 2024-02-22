function [pos, psi] = beam_conf(GRID_SIZE)
    pos = [ 1  1 1  0  0 0 0 -1 -1 -1;
           -2  0 2 -3 -1 1 3 -2  0  2;
            0  0 0  0  0 0 0  0  0  0] * GRID_SIZE;
    pos = pos - mean(pos, 2);
    psi = [0 0 0 0 0 0 0 0 0 0];
end