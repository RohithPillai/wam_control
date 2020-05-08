function [Ad_g] = calcAD_g(g)
% Get the adjoint matrix for transfromation g
    R = g(1:3,1:3);
    p = g(1:3,4);
    Ad_g = [R        matcross(p)*R;
            zeros(3)      R      ];
end 