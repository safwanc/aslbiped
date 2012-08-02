function [px, py] = ZeroMomentPoint(c, dP, dL, pz, linkm, g)
%#codegen

    persistent Mg

    if isempty(Mg)
        Mg = sum(linkm) * abs(g); 
    end

    px = (Mg*c(1) + pz*dP(1) - dL(2)) / (Mg + dP(3));
    py = (Mg*c(2) + pz*dP(2) + dL(1)) / (Mg + dP(3));
    
end