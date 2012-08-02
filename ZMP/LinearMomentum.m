function [P, Pj] = LinearMomentum(v, linkm)
%#codegen

    persistent m
    if isempty(m)
        m = linkm;
    end

    Pj = zeros(3, length(m));

    for i = 1 : length(m)
        Pj(:,i) = m(i) * v(:,i); 
    end

    P = sum(Pj, 2); 
    
end