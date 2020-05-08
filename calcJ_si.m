function J_si = calcJ_si(tws, jt, i)
    numJt = size(tws, 2);
    J_si = zeros(6,numJt);
    g = eye(4);
    J_si = sym(J_si);
    
    for j = 1:i
       z_j = tws(:, j);
       J_si(:,j) = calcAD_g(g)*z_j; %find twist'
       g = g* twistExp(z_j, jt(j));
    end
end