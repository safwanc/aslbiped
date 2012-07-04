function [U, K] = JointController(MODE, STATE, LP, RP, QE, DQE, KP, KD)
%#codegen

    global IMPACT
    persistent LAST TIMER MAX
    persistent HIPZ HIPX HIPY KNEEY ANKLEZ ANKLEY ANKLEX
	persistent QL QR ALL PITCH FOOT
    
	Kp = repmat(KP, 14, 1); 
    Kd = repmat(KD, 14, 1); 
    
    % -------------------------------------------------------------

    if isempty(HIPZ)	 HIPZ    = 	1; end
    if isempty(HIPX)	 HIPX    = 	2; end
    if isempty(HIPY)	 HIPY    = 	3; end
    if isempty(KNEEY)	 KNEEY   = 	4; end
    if isempty(ANKLEZ) 	 ANKLEZ  = 	5; end
    if isempty(ANKLEY) 	 ANKLEY  = 	6; end
    if isempty(ANKLEX) 	 ANKLEX  = 	7; end
    
    if isempty(LAST)
        LAST = struct( ...
            'STATE', STATE ...
            ); 
    end
    
    if isempty(MAX)
        MAX = 1000; 
    end
    
    if isempty(TIMER)
        TIMER = 0; 
    end

    if isempty(QL)
        QL = 1:7; 
    end
    
    if isempty(QR)
        QR = 8:14; 
    end
    
    if isempty(ALL)
        ALL = [QL QR]; 
    end
    
    if isempty(PITCH)
        PITCH = [HIPY KNEEY ANKLEY]; 
    end
    
    if isempty(FOOT)
        FOOT = [ANKLEY ANKLEX]; 
    end

    % -------------------------------------------------------------
    % MAIN FUNCTION
    % -------------------------------------------------------------
    
    SWING = QL; 
    STAND = QR; 
    
    if (MODE ~= FPEMode.Init)
        switch(STATE)
                    
            case {FPEState.LeftLift, FPEState.LeftSwing}

                Kp(SWING) = 500; 
                Kd(SWING) = 10;
                
                Kp(STAND) = 1200; 
                Kd(STAND) = 10; 
                
                Kp(STAND(FOOT)) = 2000; 
                Kd(STAND(FOOT)) = 10;
               
            case FPEState.LeftDrop
                Kp(SWING) = 500;
                Kd(SWING) = 10; 

                Kp(STAND(FOOT)) = 0; 
                Kd(STAND(FOOT)) = 0;

                if ((LP(3) < 0.01) || IMPACT)
                    Kp(SWING([KNEEY])) = 100; 
                    Kd(SWING([KNEEY])) = 2;
                    IMPACT = true; 
                end

            case FPEState.StandStill
                if IMPACT
                    
                    if LAST.STATE ~= STATE
                        TIMER = 0; 
                    end

                    TIMER = min(TIMER+1, MAX); 
                    
                    if (TIMER < MAX)
                        
                        Kp(ALL) = 500; 
                        Kd(ALL) = 10;
                        
                        Kp(SWING(PITCH)) = 80; 
                        Kd(SWING(PITCH)) = 5;
                    
                        Kp(STAND(FOOT)) = 80; 
                        Kd(STAND(FOOT)) = 5;
                    else
                        
                        IMPACT = false; 
                        TIMER = 0; 
                        
                    end

                end

        end
    end
       
    % -------------------------------------------------------------
    
    U = (Kp.*QE) + (Kd.*DQE);
    K = [Kp'; Kd']; 
    
	LAST.STATE = STATE; 
    
end