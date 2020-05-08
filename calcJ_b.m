function [Jb] =  calcJ_b(tws, jt, g, syb)
    % Calculates the body Jacobian given twists and joint angles
    Js = calcJ_s(tws, jt, syb);
    g_inv = inv(g);
    ADinv = calcAD_g(g_inv);
    Jb = ADinv*Js;    
end