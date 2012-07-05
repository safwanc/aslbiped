function [U, K] = JointController(MODE, STATE, LP, RP, QE, DQE, KP, KD)
%#codegen

    global IMPACT
    persistent LAST IMPACTCLOCK IMPACTINTERVAL IMPACTDETECTED
    persistent HIPZ HIPX HIPY KNEEY ANKLEZ ANKLEY ANKLEX
	persistent QL QR ALL PITCH FOOT
    
    SWINGFOOT = LP; 
    % @TODO: Remove LP
    
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
            'STATE', STATE, ...
            'IMPACT', IMPACT ...
            ); 
    end
    
    if isempty(IMPACTINTERVAL)
        IMPACTINTERVAL = 5000; 
    end
    
    if isempty(IMPACTCLOCK)
        IMPACTCLOCK = 0; 
    end
    
    if isempty(IMPACTDETECTED)
        IMPACTDETECTED = 0; 
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
    
    if (STATE ~= FPEState.StandStill)
        switch(STATE)

            case FPEState.LeftPush
%                                             | ___ \| | | |/  ___|| | | |
%                                             | |_/ /| | | |\ `--. | |_| |
%                                             |  __/ | | | | `--. \|  _  |
%                                             | |    | |_| |/\__/ /| | | |
%                                             \_|     \___/ \____/ \_| |_/


            case FPEState.LeftLift
%                                              | |   |_   _||  ___||_   _|
%                                              | |     | |  | |_     | |  
%                                              | |     | |  |  _|    | |  
%                                              | |_____| |_ | |      | |  
%                                              \_____/\___/ \_|      \_/ 
                Kp(SWING) = 500; 
                Kd(SWING) = 10;
                
                Kp(STAND) = 1200; 
                Kd(STAND) = 10; 
                
                Kp(STAND(FOOT)) = 2000; 
                Kd(STAND(FOOT)) = 10;

            case FPEState.LeftSwing
%                                     /  ___|| |  | ||_   _|| \ | ||  __ \
%                                     \ `--. | |  | |  | |  |  \| || |  \/
%                                      `--. \| |/\| |  | |  | . ` || | __ 
%                                     /\__/ /\  /\  / _| |_ | |\  || |_\ \
%                                     \____/  \/  \/  \___/ \_| \_/ \____/
                Kp(SWING) = 500; 
                Kd(SWING) = 10;
                
                Kp(STAND) = 1200; 
                Kd(STAND) = 10; 
                
                Kp(STAND(FOOT)) = 2000; 
                Kd(STAND(FOOT)) = 10;
                
            case FPEState.LeftDrop
                
%                                             |  _  \| ___ \|  _  || ___ \
%                                             | | | || |_/ /| | | || |_/ /
%                                             | | | ||    / | | | ||  __/ 
%                                             | |/ / | |\ \ \ \_/ /| |    
%                                             |___/  \_| \_| \___/ \_|    
%                                                                         
%     
                Kp(SWING) = 500;
                Kd(SWING) = 10; 
                
                Kp(STAND) = 100;
                Kd(STAND) = 10; 
                
                Kp(STAND(FOOT)) = 0; 
                Kd(STAND(FOOT)) = 0;

                if (~IMPACT && ~IMPACTDETECTED)
                    
                    %% PRE IMPACT  ----------------------------------------
                    if (SWINGFOOT(end) <= 0.03)
                        Kp(SWING(KNEEY)) = 0;
                        Kd(SWING(KNEEY)) = 0;
                        
%                         Kp(SWING(FOOT)) = 0;
%                         Kd(SWING(FOOT)) = 0;
                    end
                    
                elseif (IMPACT && ~IMPACTDETECTED)
                    
                    %% DETECT IMPACT  -------------------------------------
                    IMPACTDETECTED = 1;
                    IMPACTCLOCK = 0;
                    
                else
                    
                    %% DURING IMPACT  -------------------------------------
                    IMPACTCLOCK = min(IMPACTCLOCK+1, IMPACTINTERVAL); 
                    if (IMPACTCLOCK < IMPACTINTERVAL)
                        
                        % Control action to absorb large forces and hold
                        % current joint angles for a predefined period of
                        % time. 
                        
                        Kp(SWING) = KP; 
                        Kd(SWING) = 12;
                        
                        Kp(STAND) = KP; 
                        Kd(STAND) = 12;
                        
                        %Kp(SWING(PITCH)) = 500; 
                        %Kd(SWING(PITCH)) = 10;
%                     
%                         Kp(STAND(PITCH)) = 50; 
%                         Kd(STAND(PITCH)) = 10;
%                         
%                         Kp(SWING([ANKLEX ANKLEY KNEEY])) = 100; 
%                         Kd(SWING([ANKLEX ANKLEY KNEEY])) = 10;
%                         
%                         Kp(STAND([ANKLEX ANKLEY KNEEY])) = 100; 
%                         Kd(STAND([ANKLEX ANKLEY KNEEY])) = 10;
                    else
                        
                        %% POST IMPACT  -----------------------------------
                        IMPACT = false;
                    end
                end
                
            otherwise
                error('Unsupported STATE for Joint Controller');
        end
    end
       
    % -------------------------------------------------------------
    
    U = (Kp.*QE) + (Kd.*DQE);
    K = [Kp'; Kd']; 
    
	LAST.STATE = STATE; 
    LAST.IMPACT = IMPACT; 
    
end