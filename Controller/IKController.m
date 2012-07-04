function QREF = IKController(BIPED, STATE, XREF, X)
%#codegen

    global IMPACT
    persistent HOLD LAST STEP BREAKPOINT OFFSET

    TO = X(4:6); 
    Q  = X(7:20); 
    FPEX = XREF(1); 
    
    % -------------------------------------------------------------

    if isempty(HOLD)
        HOLD = struct('TO', TO, 'Q', Q); 
    end
    
    if isempty(LAST)
        LAST = struct('TO', HOLD.TO, 'QREF', HOLD.Q); 
    end
    
    if isempty(OFFSET)
        OFFSET = struct(...
            'STAND',    0.035,  ...
            'WALK',     0.000,  ...
            'NONE',     0.000   ...
            ); 
    end
    
    if isempty(STEP)
        STEP = 0; 
    end
    
    if isempty(BREAKPOINT)
        BREAKPOINT = 30; 
    end
    
    STEP = min(STEP+1, BREAKPOINT);
    
    % -------------------------------------------------------------
    % MAIN FUNCTION
    % -------------------------------------------------------------
    
	QREF = LAST.QREF; 
    
    QREF([1 5]) = 0; 
    QREF([8 12]) = 0;
    
    QREF(13) = Q(13); 
    QREF(14) = Q(14);
    
    % -------------------------------------------------------------
    
    switch(STATE)
        case FPEState.LeftDrop
            SWINGLEG = BIPED.L; 
        case FPEState.RightDrop
            SWINGLEG = BIPED.R; 
        otherwise
            error('Unsupported State for IK Controller'); 
    end
    
    
    FPEOFFSET = OFFSET.STAND;
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

    % FPE Tracking Kinematics: 
    [QREF(3), QREF(4)] = TrackPhi(BIPED.TW0, SWINGLEG, FPEX, FPEOFFSET);
    QREF(6) = - QREF(3) - QREF(4); 
    
    % -------------------------------------------------------------

	 p = TO(2) - LAST.TO(2);
     r = TO(1) - LAST.TO(1);

     QREF(3) = QREF(3) + p; 
     QREF(6) = QREF(6) + p; 
     
     QREF(2) = QREF(2) - r; 
     QREF(7) = QREF(7) - r;
     
    % -------------------------------------------------------------
    
    if (STEP == BREAKPOINT)
        STEP = 0; 
    end
    
    LAST.QREF = QREF; 
    LAST.TO = TO; 

end

function [QHIP, QKNEE] = TrackPhi(TW0, SWINGLEG, FPEX, OFFSET)
     
    % @TODO: Make a persistent struct for: 
    [LThigh, LShank, LAnkle] = LegLengths(SWINGLEG);

	HIPW = Transform(TW0, SWINGLEG.O(:,3)); 
    SWINGHIP = [HIPW(1); HIPW(3)]; 
    SWINGFOOT = [FPEX+OFFSET; SWINGLEG.FOOT.P(3)]; 
    
    [QHIP, QKNEE] = LegIK(SWINGHIP-SWINGFOOT, LThigh, LShank+LAnkle);

end

function [QHip, QKnee] = LegIK(P, LThigh, LShank)

    X = P(1); 
    Z = P(end); 

    D = ((X^2) + (Z^2) - (LShank^2) - (LThigh^2)) / (2*LShank*LThigh); 
    
    if (D > 1) 
        % This case occurs when the target is out of the workspace reach
        QKnee = 0; 
        
    else
        QKnee = atan2(sqrt(1-(D^2)), D);
        
        if (QKnee < 0)  
            % Always pick the +ve knee angle solution. 
            QKnee = atan2(-sqrt(1-(D^2)), D);
        end
    end
    
    
    QHip = atan(X/Z) - atan( ...
        (LShank * sin(QKnee)) / ...
        (LThigh + LShank*cos(QKnee)) ...
        );

end




function [LThigh, LShank, LAnkle] = LegLengths(LEG)

    OHip = LEG.O(:,3); 
    OKnee = LEG.O(:,4); 
    OAnkle = LEG.O(:,6); 

    LThigh = norm(OHip-OKnee,2); 
    LShank = norm(OKnee-OAnkle,2); 
    LAnkle = 0.06480;
    
end

function [ P0 ] = Transform(T0W, PW)

    P = T0W * [PW; 1]; 
    P0 = P(1:3); 
    
end