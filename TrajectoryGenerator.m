function [ XCOM, XLP, XRP ] = ...
    TrajectoryGenerator( STATE, COM, FPEX, LP, RP, ...
    STEPL, STEPW, COMH, GNDH, GNDCLR)
    %#codegen

    % Generator State Variables
    persistent LAST
    if isempty(LAST)
        LAST = struct( ...
            'STATE', STATE, ...
            'XCOM', [0.0329 -0.0198 0.402]', ...
            'XLP', [-0.0273 -0.1292 -0.001627]', ...
            'XRP', [-0.02575 0.13 -0.002]' ...
            ); 
    end
    
    if (STATE ~= LAST.STATE)
        %
        
        % [ SWING, STAND ] = ParseState(STATE, LP, RP);
        SWING = LP; STAND = RP; 
        
        switch(STATE)
            case {FPEState.LeftPush, FPEState.RightPush}
                XCOM = [STAND(1) STAND(2) COMH]'; 
            otherwise
                error('Unsupported State'); 
        end
        
        XLP = SWING; XRP = STAND; 
        % [ XLP, XRP ] = SetState(STATE, SWING, STAND);
        
        LAST.XCOM = XCOM; 
        LAST.XLP = XLP; 
        LAST.XRP = XRP; 
        
    else
        
        % @TODO: Check if in DROP state (to keep updating swing foot traj
        %           in order to track FPE.
        
        XCOM = LAST.XCOM; 
        XLP = LAST.XLP; 
        XRP = LAST.XRP; 
    end

    LAST.STATE = STATE;

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
