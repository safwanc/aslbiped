function [ XCOM, XLP, XRP ] = ...
    TrajectoryGenerator( STATE, COM, FPEX, LP, RP, ...
    STEPL, STEPW, COMH, GNDH, GNDCLR)
    %#codegen

    % Generator State Variables
    persistent LAST
    if isempty(LAST)
        LAST = struct( ...
            'STATE', STATE, ...
            'TIMER', 0, ...
            'XCOM', COM, ...
            'XLP', LP, ...
            'XRP', RP ...
            ); 
    end
    
    persistent UPDATE
    if isempty(UPDATE)
        UPDATE = 0; 
    end
    
    persistent HOLD
    if isempty(HOLD)
        HOLD = struct( ...
            'STAND', zeros(3,1), ...
            'SWING', zeros(3,1), ...
            'COM', zeros(3,1)    ...
        ); 
    end
    
    if (STATE ~= LAST.STATE)
        
        if ((STATE == FPEState.LeftPush) || (STATE == FPEState.LeftLift) ||  (STATE == FPEState.LeftSwing) || (STATE == FPEState.LeftDrop))
            SWING = LP; STAND = RP; 
        else
            SWING = RP; STAND = LP; 
        end
                
        switch(STATE)
            case {FPEState.StandStill}
                XCOM = COM; 
            
            case {FPEState.LeftPush, FPEState.RightPush}
                HOLD.COM = COM; 
                
                XCOM = [STAND(1) STAND(2) COMH]'; 
                SWING(3) = 0; 
                STAND(3) = GNDH*2; 
                
                HOLD.STAND = STAND; 
                HOLD.SWING = SWING; 
                               
                
            case {FPEState.LeftLift, FPEState.RightLift}
                
                SWING = HOLD.SWING; 
                STAND = HOLD.STAND; 
                XCOM = LAST.XCOM;
                
                SWING(3) = GNDCLR;
                HOLD.SWING = SWING; 
                
            case {FPEState.LeftSwing, FPEState.RightSwing}
                
                SWING = HOLD.SWING; 
                STAND = HOLD.STAND; 
                XCOM = LAST.XCOM; 
                XCOM(1) = XCOM(1) + 2*0.085;
                XCOM(2) = XCOM(2) - 0.035;
                
                SWING(1) = SWING(1) + 2*STEPL; 
                HOLD.SWING = SWING; 
                
            case {FPEState.LeftDrop, FPEState.RightDrop}
                SWING = HOLD.SWING;
                STAND = HOLD.STAND; 
                XCOM = LAST.XCOM;
                XCOM(1) = XCOM(1) + 0.085;
                
                SWING(3) = 0; 
                
                HOLD.SWING = SWING; 
                
                % LAST.TIMER = 0; 
                
            otherwise
                error('Unsupported State'); 
        end
        
        if ((STATE == FPEState.LeftPush) || (STATE == FPEState.LeftLift) ||  (STATE == FPEState.LeftSwing) || (STATE == FPEState.LeftDrop))
            XLP = SWING; XRP = STAND; 
        else
            XLP = STAND; XRP = SWING;
        end
        
        LAST.XCOM = XCOM; 
        LAST.XLP = XLP; 
        LAST.XRP = XRP; 
        % LAST.TIMER = 0; 
    else
        
        XCOM = LAST.XCOM; 
        XLP = LAST.XLP; 
        XRP = LAST.XRP; 
        
    end

    LAST.STATE = STATE;
    %LAST.TIMER = LAST.TIMER + 1; 
    
    %TIMER = LAST.TIMER; 

end

%     XCOM = [0.0329 -0.0198 0.402]'; 
%     XLP = [-0.0273 -0.1292 -0.001627]';
%     XRP = [-0.02575 0.13 -0.002]'; 
    
%     [ SWING, STAND ] = ParseState(STATE, LP, RP);
%     
%     switch(STATE)
%         case PUSH
%             XCOM = [STAND(1) STAND(2) COMH]'; 
%         case LIFT
%             XSWING = [STAND(1) SWING(2) GNDCLR]'; 
%         case SWING
%             XSWING = [SWING(1)+STEPL SWING(2) GNDCLR]'; 
%         case DROP
%             XSWING = [FPEX STAND(2)+STEPW GNDH]';
%             XCOM = [STAND(1)+(STEPL/2) STAND(2)+(STEPW/2) COMH]'; 
%         otherwise
%             error('Unsupported State'); 
%     end
%     
%     [ XLP, XRP ] = SetState(STATE, SWING, STAND)

function [ SWING, STAND ] = ParseState(STATE, LP, RP)

    switch(STATE)
        case {1, 2, 3, 4}
            SWING = LP; 
            STAND = RP; 
        case {5, 6, 7, 8}
            SWING = RP; 
            STAND = LP; 
        otherwise
            error('Unsupported State'); 
    end

end

function [ LP, RP ] = SetState(STATE, SWING, STAND)

    switch(STATE)
        case {1, 2, 3, 4}
            LP = SWING; 
            RP = STAND; 
        case {5, 6, 7, 8}
            RP = SWING; 
            LP = STAND; 
        otherwise
            error('Unsupported State'); 
    end

end
