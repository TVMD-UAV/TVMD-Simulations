function pB = pseudo_inverse(B)
    pB = B' / (B * B');
end
