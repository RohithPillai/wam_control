function g = calcGi(tws, jt, gsli0, i)
    g =  gsli0;
    for j = i:-1:1
        g = twistExp(tws(:,j), jt(j))*g;
    end
end