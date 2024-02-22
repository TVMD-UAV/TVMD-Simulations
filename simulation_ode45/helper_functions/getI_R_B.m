function I_R_B = getI_R_B(psi, phi, theta)
    I_R_B = Rz(psi) * Rx(phi) * Ry(theta);
end