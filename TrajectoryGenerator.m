function [ XCOM, XLP, XRP ] = TrajectoryGenerator( STATE, COM, FPEX, LP, RP )
%#codegen

    persistent BETA STEPL STEPW COMH GNDH GNDCLR
    if isempty(BETA) 
        BETA = 24 * pi/180; 
    end
    
    if isempty(STEPL)
        STEPL = 0.17; 
    end
    
    if isempty(STEPW)
        STEPW = 0.1640; 
    end
    
    if isempty(COMH)
        COMH = 0.4; 
    end
    
    if isempty(GNDH)
        GNDH = -0.002; 
    end
    
    if isempty(GNDCLR)
        GNDCLR = 0.07; 
    end
    
    % @TODO
    SWING = LP; 
    STAND = RP; 
    
    switch(STATE)
        case PUSH
            XCOM = [STAND(1) STAND(2) COMH]'; 
        case LIFT
            XSWING = [STAND(1) SWING(2) GNDCLR]'; 
        case SWING
            XSWING = [SWING(1)+STEPL SWING(2) GNDCLR]'; 
        case DROP
            XSWING = [FPEX STAND(2)+STEPW GNDH]';
            XCOM = [STAND(1)+(STEPL/2) STAND(2)+(STEPW/2) COMH]'; 
        otherwise
            error('Unsupported State'); 
    end
    
    XLP = XSWING; 
    XRP = RP; 

end