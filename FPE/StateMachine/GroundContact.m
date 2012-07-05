function c  = GroundContact(sw)
    global IMPACT
    persistent IMPACTDETECTED
    
    if isempty(IMPACTDETECTED)
        IMPACTDETECTED = 0; 
    end
    
    c = false;
    
    if IMPACT
        if ~IMPACTDETECTED
            IMPACTDETECTED = 1; 
        end
    end
    
%    if (~IMPACT && IMPACTDETECTED && (sw(end) <= 0))
    if (~IMPACT && IMPACTDETECTED)
        c = true;
        IMPACTDETECTED = 0; 
    end
    
end
