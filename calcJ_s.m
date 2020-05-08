function [J] =  calcJ_s(tws, jt, syb)
    % Calculates the Jacobian given twists and joint angles
    numJt = size(tws, 2);
    J = nan(6,numJt);
    if syb
        J = sym(nan(6,numJt));
    end
    g = eye(4);
    
    for i = 1:numJt
       z_i = tws(:, i);
       J(:,i) = calcAD_g(g)*z_i; %find twist'
       g = g* twistExp(z_i, jt(i));
    end
    
end