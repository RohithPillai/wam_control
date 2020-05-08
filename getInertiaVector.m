function InVec = getInertiaVector(Q, I)
    InMat = Q*diag(I)*Q';
    InVec = [InMat(1,1), InMat(2,2), InMat(3,3), InMat(2,3), InMat(1,3),InMat(1,2)]; 
end