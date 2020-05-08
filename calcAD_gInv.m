function [Ad_g_inv] = calcAD_gInv(g)
% Get the adjoint matrix for transfromation g
    R = g(1:3,1:3);
    p = g(1:3,4);
    Ad_g_inv = [R'        -R'*matcross(p);
            zeros(3)      R'      ];
end 