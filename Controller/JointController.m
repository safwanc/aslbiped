function [U, K] = JointController(MODE, STATE, LP, RP, QE, DQE, KP, KD)
%#codegen

    Kp = repmat(KP, 14, 1); 
    Kd = repmat(KD, 14, 1); 
    
    persistent HIPZ HIPX HIPY KNEEY ANKLEZ ANKLEY ANKLEX
	persistent QL QR ALL ROLL PITCH FOOT DEBUGSTEP BREAKPOINT
    persistent LAST IMPACT TIMER MAX KPT KDT MAXT

 % -------------------------------------------------------------
    if isempty(LAST)
        LAST = struct( ...
            'STATE', STATE ...
            ); 
    end
    
	if isempty(QL)       QL = 1 : 7;    end
    if isempty(QR)       QR = 8 : 14;   end
    if isempty(HIPZ)	 HIPZ    = 	1;  end
    if isempty(HIPX)	 HIPX    = 	2;  end
    if isempty(HIPY)	 HIPY    = 	3;  end
    if isempty(KNEEY)	 KNEEY   = 	4;  end
    if isempty(ANKLEZ) 	 ANKLEZ  = 	5;  end
    if isempty(ANKLEY) 	 ANKLEY  = 	6;  end
    if isempty(ANKLEX) 	 ANKLEX  = 	7;  end
    
    if isempty(ALL)     ALL = [QL QR];                  end
    if isempty(ROLL)    ROLL = [HIPX ANKLEX];           end
    if isempty(PITCH)   PITCH = [HIPY KNEEY ANKLEY];    end
    if isempty(FOOT)    FOOT = [ANKLEY ANKLEX];         end
    
    if isempty(DEBUGSTEP)   DEBUGSTEP = 0;      end
    if isempty(BREAKPOINT)  BREAKPOINT = 30;    end
    
    if isempty(MAX)     MAX = 1000;     end
    if isempty(TIMER)   TIMER = 0;      end
    if isempty(IMPACT)  IMPACT = 0;     end
    if isempty(MAXT)    MAXT = 5000;	end
    
    if isempty(KPT)      KPT = SmoothTrajectory(KP, KP, MAXT); end
    if isempty(KDT)      KDT = SmoothTrajectory(KD, KD, MAXT); end
    
    % -------------------------------------------------------------
    % MAIN FUNCTION
    % -------------------------------------------------------------
    
    if ((STATE == FPEState.LeftPush) || (STATE == FPEState.LeftLift) ||  (STATE == FPEState.LeftSwing) || (STATE == FPEState.LeftDrop))
        SWING = QL; STAND = QR;
    else
        SWING = QR; STAND = QL;
    end
    
    % @TUNING (Global Case) ----------------------

	% --------------------------------------------
    
    switch(STATE)
        case {FPEState.LeftPush, FPEState.RightPush}
            % @TUNING ------------------------------------
            %Kp(STAND([HIPZ HIPX HIPY KNEEY])) = KP/4; 
            %Kd(STAND([HIPZ HIPX HIPY KNEEY])) = KD/1; 
            % --------------------------------------------
            
        case {FPEState.LeftLift, FPEState.LeftSwing, ...
                FPEState.RightLift, FPEState.RightSwing}
            
            Kp(SWING) = 500;
            Kd(SWING) = 10;
            
            % @TUNING ------------------------------------
            
            % --------------------------------------------
            
        case {FPEState.LeftDrop, FPEState.RightDrop}
            
            if (LAST.STATE ~= STATE)
                TIMER = 0; 
                KPT = SmoothTrajectory(...
                    [500 500 500 500 500 500 500], ...
                    [KP KP KP KP KP 500 500], ...
                    MAXT);
                KDT = SmoothTrajectory(...
                    [10 10 10 10 10 10 10], ...
                    [KD KD KD KD KD KD KD], ...
                    MAXT);
            end
            
            TIMER = min(TIMER+1, MAXT); 
            
            Kp(SWING) = KPT(TIMER, :); 
            Kd(SWING) = KDT(TIMER, :); 
    end
    
    U = (Kp.*QE) + (Kd.*DQE);
    K = [Kp'; Kd']; 
    
	LAST.STATE = STATE; 
    
end

% ------------------------------------------------------------------------
% UTILITY FUNCTIONS
% ------------------------------------------------------------------------

function [qt] = SmoothTrajectory(q0, q1, tv)
    tv = min(tv, 100000); 
    tscal = 1;
    t = (0:(tv-1))'/(tv-1);
    q0 = q0(:);
    q1 = q1(:);
    qd0 = zeros(size(q0));
    qd1 = qd0;

    A = 6*(q1 - q0) - 3*(qd1+qd0)*tscal;
    B = -15*(q1 - q0) + (8*qd0 + 7*qd1)*tscal;
    C = 10*(q1 - q0) - (6*qd0 + 4*qd1)*tscal;
    E = qd0*tscal;
    F = q0;

    tt = [t.^5 t.^4 t.^3 t.^2 t ones(size(t))];
    c = [A B C zeros(size(A)) E F]';
    
    qt = tt*c;
end

% ////////////////////////////////////////////////////////////////////////
% ////////////////////////////////////////////////////////////////////////