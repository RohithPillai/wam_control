function g = calcG(tws, jt, gst0)
    g =  gst0;
    for i = size(tws,2):-1:1
        g = twistExp(tws(:,i), jt(i))*g;
    end
end