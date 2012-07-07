function QREF = IKController(BIPED, STATE, XREF, X)
%#codegen

    global IMPACT
    persistent HOLD LAST DEBUGSTEP BREAKPOINT FPEOFFSET 
    persistent IMPACTDETECTED IMPACTCLOCK UPDATEINTERVAL
    
    TO = X(4:6); 
    Q  = X(7:20); 
    TW0 = BIPED.TW0; 
    COM = BIPED.COM; 
    FPEX = XREF(1); 
    
    % -------------------------------------------------------------
    % MAIN FUNCTION
    % -------------------------------------------------------------
    
    if isempty(HOLD)
        HOLD = struct('TO', TO, 'Q', Q); 
    end
    
    if isempty(LAST)
        LAST = struct('TO', HOLD.TO, 'QREF', HOLD.Q); 
    end
    
    if isempty(FPEOFFSET)
        OFFSET = struct(...
            'STAND',    0.035,  ...
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
            FPEOFFSET = OFFSET.NONE;
    end
    
    if isempty(DEBUGSTEP)
        DEBUGSTEP = 0; 
    end
    
    if isempty(BREAKPOINT)
        BREAKPOINT = 30; 
    end
    
    if isempty(IMPACTDETECTED)
        IMPACTDETECTED = 0; 
    end
    
    if isempty(UPDATEINTERVAL)
        UPDATEINTERVAL = 500; 
    end
    
    if isempty(IMPACTCLOCK)
        IMPACTCLOCK = 0; 
    end
    
    DEBUGSTEP = min(DEBUGSTEP+1, BREAKPOINT);
    
    % -------------------------------------------------------------
    % STATE/MODE DEPENDENT
    % -------------------------------------------------------------
    
    switch(STATE)
        case FPEState.LeftDrop
            SWINGLEG = BIPED.L;
            STANDLEG = BIPED.R;
        case FPEState.RightDrop
            SWINGLEG = BIPED.R;
            STANDLEG = BIPED.L;
        otherwise
            error('Unsupported STATE for IK Controller');
    end
    
	% -------------------------------------------------------------
    % MAIN FUNCTION
    % -------------------------------------------------------------
    
    
    if ~IMPACTDETECTED
        
        if GroundContact(SWINGLEG)
            
            %% DETECT IMPACT  -------------------------------------
            
            IMPACT = true; 
            IMPACTDETECTED = 1; % to avoid race conditions
            
            HOLD.Q = Q;
            HOLD.TO = TO; 
            
            % @EXPERIMENT: Fix Torso Orientation: 
            %HOLD.Q = FixOrientation(TO, Q); 
            
            QREF = HOLD.Q;
            
        else
            
            %% PRE IMPACT  -----------------------------------------------
            QREF = LAST.QREF;
            
            % Orientation Compensation
            % .. Keeps swing foot sagittal plane aligned with stance foot 
            % .. while FPE is being tracked.
            [TORSOROLL, TORSOPITCH, TORSOYAW] = TorsoOrientation(STANDLEG);

            % SWING LEG  
            QREF(1) = -TORSOYAW; 
            QREF(2) = -TORSOROLL;
            QREF(7) =  TORSOROLL;
            
            % FPE Tracking Kinematics (Sagittal Plane)
            [QREF(3), QREF(4)] = SagittalIK(STANDLEG, SWINGLEG, ...
                TW0, FPEX+FPEOFFSET, COM)
            
            QREF(6) = - QREF(3) - QREF(4) - TORSOPITCH;
            
            % Lock ANKLEYAW joint
            QREF(5) = 0; 
        end
        
    else
        
        %% DURING IMPACT  -------------------------------------
        IMPACTCLOCK = min(IMPACTCLOCK+1, UPDATEINTERVAL); 
        
        HOLD.Q = Q; 
        % @EXPERIMENT: Fix Torso Orientation:
        %HOLD.Q = FixOrientation(TO, Q);
        % Hold joint values until contact forces stabilize. 
        QREF = HOLD.Q;
        
        
        
        % @EXPERIMENT: Update at regular intervals? 
        if (IMPACTCLOCK == UPDATEINTERVAL) 
            IMPACTCLOCK = 0; 
            %HOLD.Q(8:14) = Q(8:14); 
        end
        
    end
    
    % -------------------------------------------------------------
    
    if (DEBUGSTEP == BREAKPOINT)
        DEBUGSTEP = 0; 
    end
    
    LAST.QREF = QREF; 
    LAST.TO = TO; 

end

function [TORSOROLL, TORSOPITCH, TORSOYAW] = TorsoOrientation(STANDLEG)

    R0F = STANDLEG.T0N(1:3,1:3,end); 
    TF = GetRPY(R0F'); 
    
    TORSOROLL   = TF(1); 
    TORSOPITCH  = TF(2); 
    TORSOYAW    = TF(3); 
    
end

function [ QREF ] = FixOrientation(TO, Q)
    % @UNFINISHED
    R = abs(TO(1)); 
    P = TO(2); 
    Y = TO(3); 
    
    QREF = Q; 
    QREF(2) = QREF(2) - R; 
    QREF(7) = QREF(7) + R; 
    QREF(9) = QREF(9) + R; 
    QREF(14) = QREF(9) - R; 
    
%     QREF(2) = QREF(2) + R; 
%     QREF(7) = QREF(7) - R; 
%     
%     QREF(9) = QREF(9) - R; 
%     QREF(14) = QREF(14) + R; 
    

end

function [QHIP, QKNEE] = SagittalIK(STANDLEG, SWINGLEG, TW0, FPEW, COMW)

    persistent L TANKLE
    if isempty(L) L = 0.411394485631267; end
    if isempty(TANKLE) TANKLE = [eye(3) [0 0 -0.06480]'; 0 0 0 1]; end

    % @TODO: Make a persistent struct for: 
    [LThigh, LShank, LAnkle] = LegLengths(SWINGLEG);
    %COMH = 0.400840121117681; HIPH = 0.634317429238331; 
    
    % -------------------------------------------------------------

    T0F = STANDLEG.T0N(:,:,end) * TANKLE; 

    TWF = TW0 * T0F; 
    TFW = inv(TWF); 
    
    T0H = SWINGLEG.T0N(:,:,2); TFH = T0F\T0H;
    

    
    FPEF = Transform(TFW, [FPEW 0 0]'); 
    FPEX = FPEF(1); 
    
    HIPF = TFH(1:3,4); HIPX = HIPF(1); HIPZ = HIPF(3); 
    
    % -------------------------------------------------------------
    % APPROACH #1: Doesn't appear to be work, tracks FPE too fast
    % COMF = Transform(TFW, COMW); COMX = COMF(1); COMZ = COMF(3); 
    % FPEZ = COMZ - sqrt( (L^2) - ((FPEX-COMX)^2) ); 
    % 
    
    % APPROACH #2:
    TSW = T0F\(SWINGLEG.T0N(:,:,end) * TANKLE);
    FPEZ = TSW(3,4); % Z-Component in 4x4 Transform
    % -------------------------------------------------------------
    
    % IK in fixed foot sagittal plane
    SWINGHIP = [HIPX; HIPZ]; 
    SWINGFOOT = [FPEX; FPEZ]; 
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

function [ c ] = GroundContact(LEG)

    persistent GNDTOL 
    if isempty(GNDTOL)
        GNDTOL = -0.001; 
    end

    c = 0; 
    
    if all(LEG.FOOT.CP(3,:) < GNDTOL)
        c = 1; 
    end

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

function [ RPY ] = GetRPY(R)

    if ((R(1,1) == 0) || R(3,3) == 0)
        error('Degenerate Case'); 
    end

    Roll    = atan2(R(3,2), R(3,3)); 
    Pitch   = atan2(-R(3,1), hypot(R(3,2), R(3,3))); 
    Yaw     = atan2(R(2,1), R(1,1)); 
    
    RPY = [Roll; Pitch; Yaw]; 
    
end