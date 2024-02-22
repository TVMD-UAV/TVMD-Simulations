function pB = pseudo_inverse_tall(B)
    pB = (B' * B) \ B';
end
