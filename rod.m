function R = rod(omega, theta)
    R = eye(3) + matcross(omega)*sin(theta) + (matcross(omega)*matcross(omega))*(1-cos(theta));
end