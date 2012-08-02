function R = GetRotationMatrices(T0X)
%#codegen

    T = zeros(4,4,15); 
    TColumns = reshape(T0X, 16, 15); 
    
    for i = 1 : 15
        T(:,:,i) = reshape(TColumns(:,i), 4, 4); 
    end
    
    R = squeeze(T(1:3,1:3,:)); 

end