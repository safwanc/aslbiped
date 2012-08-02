function L = AngularMomentum(Pj, c, w, R, linkI)
%#codegen

    persistent I
    if isempty(I)
        I = linkI; 
    end

    Lj = zeros(3, size(I,3));
    
    for i = 1 : length(I)
        Lj(:,i) = cross(c(:,i), Pj(:,i)) + (R(:,:,i)* I(:,:,i)* R(:,:,i)') * w(:,i);
    end

    L = sum(Lj, 2); 

end