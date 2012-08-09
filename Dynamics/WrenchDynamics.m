function W = WrenchDynamics(BIPED)
%#codegen
    
    W = -[ContactWrench(BIPED.L.FOOT) ContactWrench(BIPED.R.FOOT)];
    
end

function [ W ] = ContactWrench(FOOT)

    f = zeros(3,1); 
    h = zeros(3,1); 
    
    for i = 1 : 4
        f = f + FOOT.CF(:,i); 
        h = h + cross(FOOT.CP(:,i), FOOT.CF(:,i)); 
    end
    
    W = [f; h]; 

end