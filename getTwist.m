
function [tw] = getTwist(w, q)
    % calculates twists for revolute joints
    % takes in the w and q point along it's axis to calculate the twist 
    tw =[cross(-w,q); w];
end