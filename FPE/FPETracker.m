function P = FPETracker(BIPED, STATE, FPEX, PHI, STEPW)
%#codegen

    persistent FPEOFFSET
    
    if isempty(FPEOFFSET)
        OFFSET = struct(...
            'STAND',    0.035,  ...
            'DEBUG',    0.080,  ...
            'WALK',    -0.135,  ...
            'NONE',     0.000   ...
            ); 
        
            %{
            switch(MODE)
                case FPEMode.Stand
                    FPEOFFSET = OFFSET.STAND;
                case FPEMode.Walk
                    FPEOFFSET = OFFSET.STAND;
                otherwise
                    FPEOFFSET = OFFSET.STAND;
            end
            %} 
            
            % @TODO: Remove hard coded offset once MODE is an input
            FPEOFFSET = OFFSET.DEBUG;
    end

    P = zeros(3,1); 

    % -------------------------------------------------------------
    % STATE/MODE DEPENDENT
    % -------------------------------------------------------------
    
    switch(STATE)
        case FPEState.LeftDrop
            SWINGLEG = BIPED.L;
            STANDLEG = BIPED.R;
            STEPY = STANDLEG.FOOT.P(2) - STEPW; 
            
        case FPEState.RightDrop
            SWINGLEG = BIPED.R;
            STANDLEG = BIPED.L;
            STEPY = STANDLEG.FOOT.P(2) + STEPW; 
            
        otherwise
            error('Unsupported STATE for IK Controller');
    end

    P(1) = FPEX + FPEOFFSET; 
    P(2) = STEPY; 
    P(3) = 0; 

end