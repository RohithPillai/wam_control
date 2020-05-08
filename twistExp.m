function x = twistExp(twist, theta)
    v = twist(1:3);
    omega = twist(4:6);
    x = [rod(omega, theta), ((eye(3)-rod(omega, theta))*cross(omega,v))...
        + (omega*dot(omega,v)*theta); 0, 0, 0, 1];
end