function [ N ] = NullSpaceProjection(J)
    
    [~,n] = size(J); 
    N = eye(n,n) - (Inverse(J) * J); 

end