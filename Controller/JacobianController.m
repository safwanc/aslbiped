function DQ = JacobianController(BIPED, STATE, DX)
%#codegen

    DQ = zeros(14,1); 

    % Controller State Variables
    persistent LAST
    if isempty(LAST)
        LAST = struct( ...
            'STATE', STATE ...
            ); 
    end

    
    persistent DEBUGSTEP BREAKPOINT
    if isempty(DEBUGSTEP)
        DEBUGSTEP = 0;
    end
    
    if isempty(BREAKPOINT)
        BREAKPOINT = 30; 
    end
    
    
    % Constant Variables (Optimization)
    persistent L R LR S JB I
    if isempty(L)   L = 1; end
    if isempty(R)   R = 2; end
    if isempty(LR) LR = 0; end
    if isempty(I)   I = eye(20); end
    if isempty(S)   S = [eye(14) zeros(14, 6)]; end
    if isempty(JB) JB = [zeros(6,14) eye(6)];   end
    
    DEBUGSTEP = min(DEBUGSTEP+1, BREAKPOINT);
    
    %% COMMON CALCULATIONS (STATE INDEPENDENT): 
    
    [ SWING, STAND ] = ParseState(STATE);
    
    switch(SWING)
        case 1
            [ DXcom_xy, DXcom_z, ~, DXswing, ~] = ParseDX(BIPED, DX);
        case 2
            [ DXcom_xy, DXcom_z, ~, ~, DXswing] = ParseDX(BIPED, DX);
        otherwise
            error('Unable to parse DX'); 
    end
    
    [ Jcom_xy, Jcom_z ] = SplitJcom(Jcom(BIPED)); 
    Jswing = Jleg(SWING, BIPED);
    
    
    %% STATE SPECIFIC CONTROL ACTION: 
    
    switch(STATE)

         case {FPEState.LeftLift, FPEState.RightLift}
            
             
            DXswing(3) = 4*DXswing(3);
            %DXswing(4:6) = 1*DXswing(4:6);
            
            %% High Priority Tasks
            Jhigh   = Jcom_xy;
            DXhigh  = DXcom_xy;  
            [Jh, DXh] = Jconstraint(STAND, BIPED, Jhigh, DXhigh); 
            
            Jhdebug = Inverse(Jh);
            Jhdebug(6,:) .* DXh'
            
            %% Low Priority Tasks
            Jl  = Jswing; 
            DXl = DXswing; 
            
            DQ(:,:) = S * Prioritized(Jh, DXh, Jl, DXl); 
            
            % DEBUG
            
            Jldebug = Inverse(Jl);
            Jldebug(6,:) .* DXl'
            
            if (DEBUGSTEP == BREAKPOINT)
                DEBUGSTEP = 0;
            end

         case {FPEState.LeftSwing, FPEState.RightSwing}
            
            
            %% High Priority Tasks
            Jhigh   = Jcom_xy;
            DXhigh  = DXcom_xy;  
            [Jh, DXh] = Jconstraint(STAND, BIPED, Jhigh, DXhigh); 
            
            %% Low Priority Tasks
            Jl  = Jswing; 
            DXl = DXswing; 
            
            DQ(:,:) = S * Prioritized(Jh, DXh, Jl, DXl); 
            
            
        case {FPEState.LeftPush, FPEState.RightPush}

            [Jh, DXh] = Jdouble(BIPED, Jcom_xy, DXcom_xy); 
            DQ(:,:) = S * Inverse(Jh) * DXh; 
            
        case {FPEState.LeftDrop, FPEState.RightDrop}
            
            Jhigh   = [Jcom_xy; Jswing];
            DXhigh  = [DXcom_xy; DXswing];
            [Jh, DXh] = Jconstraint(STAND, BIPED, Jhigh, DXhigh); 
            
            DQ(:,:) = S * Inverse(Jh) * DXh; 
            
        otherwise
           
            Kcom = 1;
            if STATE == FPEState.LeftPush
                Kcom = 6; 
            end

            DXcom_xy(1) = Kcom*DXcom_xy(1);

            %% High Priority Tasks
            Jhigh   = [Jcom_xy; Jswing];
            DXhigh  = [DXcom_xy; DXswing];
            [Jh, DXh] = Jconstraint(STAND, BIPED, Jhigh, DXhigh); 
            
            %% Low Priority Tasks
            Jl  = Jcom_z; 
            DXl = DXcom_z; 
            
            DQ(:,:) = S * Prioritized(Jh, DXh, Jl, DXl); 
    end
    
    LAST.STATE = STATE; 
end

function [ DXcom_xy, DXcom_z, DXtorso_rpy, DXleft, DXright] = ParseDX(BIPED, DX)

    DXcom_xy = DX(1:2); 
    DXcom_z = DX(3); 
    DXtorso_rpy = DX(4:6); 
%     DXleft = DX(7:12); 
%     DXright = DX(13:18); 
	DXleft  = [DX(7:9);     FixOrientation(BIPED.L.FOOT.O, DX(10:12))];  
    DXright = [DX(13:15);   FixOrientation(BIPED.R.FOOT.O, DX(16:18))];  
    

end

function [ W ] = FixOrientation(o, w)

    psi = o(1); 
    theta = o(2); 
    phi = o(3); 

    dpsi = w(1); 
    dtheta = w(2); 
    dphi = w(3); 

    W = zeros(3,1); 
    W(1) = (cos(phi) * cos(theta) * dpsi) - (sin(phi) * dtheta); 
    W(2) = (cos(phi)*dtheta) + (cos(theta)*sin(phi)*dpsi); 
    W(3) = dphi - (sin(theta)*dpsi); 

end

function [ SWING, STAND ] = ParseState(STATE)

    switch(STATE)
        case {STATE.LeftPush, STATE.LeftLift, STATE.LeftSwing, STATE.LeftDrop}
            SWING = 1; 
            STAND = 2; 
        case {STATE.RightPush, STATE.RightLift, STATE.RightSwing, STATE.RightDrop}
            SWING = 2; 
            STAND = 1; 
        otherwise
            error('Unsupported State'); 
    end

end

function [ DQ ] = Prioritized(J1, X1, J2, X2)

    DQ = Inverse(J1)*X1 + (Inverse(J2*Null(J1))*(X2-(J2*Inverse(J1)*X1))); 

end

% ------------------------------------------------------------------------
% SINGLE SUPPORT CONSTRAINTS
% ------------------------------------------------------------------------

function [ Jc, DXc ] = Jconstraint(SIDE, BIPED, J, DX)
    
    
    DXs = zeros(6,1);
    
     switch(SIDE)
         
        case 1  % Left Foot Single Support 
            Js = Jleg(SIDE, BIPED); 
            if ~InContact(BIPED.R.FOOT)
                DXs = [BIPED.L.FOOT.V; BIPED.L.FOOT.W]; 
            end
            
        case 2 % Right Foot Single Support 
            Js = Jleg(SIDE, BIPED); 
            if ~InContact(BIPED.L.FOOT)
                DXs = [BIPED.R.FOOT.V; BIPED.R.FOOT.W]; 
            end
            
         otherwise
             Js1 = Jleg(1, BIPED); 
             Js2 = Jleg(2, BIPED); 
             
             Js = [Js1(1:3,:); Js2(1:3,:)]; 
             %DXs = zeros(size(Js,1),1);
             
     end

    if (rank(Js(:,15:20)) ~= 6)
        error('Constraint Jacobian Lost Rank'); 
    end

    DXc = [DXs; DX];
	Jc  = [Js; J];
    
end


function [ Jc, DXc ] = Jdouble(BIPED, J, DX)

    Js1 = Jleg(1, BIPED);
    Js2 = Jleg(2, BIPED);

    Js = [Js1(1:6,:); Js2(1:6,:)];
    DXs = zeros(size(Js,1),1);

    if (rank(Js(:,15:20)) ~= 6)
        error('Constraint Jacobian Lost Rank'); 
    end

    DXc = [DXs; DX];
	Jc  = [Js; J];
    
end

function [ c ] = InContact(FOOT)

    c = 0; 
    
    if sum(FOOT.CF(end,:)) > 0
        c = 1; 
    end

end

function [ b ] = SingleSupportStable(COM, STANCELEG)

    b = 1; 

    XCP = STANCELEG.FOOT.CP(1,:); 
    YCP = STANCELEG.FOOT.CP(2,:); 
    
    xmax = max(XCP); ymax = max(YCP); 
    xmin = min(XCP); ymin = min(YCP); 
    
    if (...
            (COM(1) > xmax) || (COM(1) < xmin) ...
         || (COM(2) > ymax) || (COM(2) < ymin) ...
        )
        b = 0; 
    end

end

% ------------------------------------------------------------------------
% COM JACOBIANS
% ------------------------------------------------------------------------

function [ J ] = Jcom(BIPED)

    J = zeros(3,20); 
    
    [RB, PB] = Decompose(BIPED.TW0); 
    
    % Leg Contributions
    J(:,1:7)  = RB * LegJCOM(BIPED.L);      % Left Leg
    J(:,8:14) = RB * LegJCOM(BIPED.R);      % Right Leg
    
    % Base Contribution
    J(:,15:17) = eye(3);
    J(:,18:20) = Rcross(RB, BIPED.COM-PB); 
    
end

function [ J ] = LegJCOM(LEG)

    J = zeros(3,7); 
    
    persistent W
    if isempty(W) W = LegWeights(LEG.M); end

	Z = LEG.Z;                  % Joint axis w.r.t base
    O = LEG.O;                  % Joint origins w.r.t. base
    XC = LEG.XC;                % COM position w.r.t. base
    P = LegPCOM(LEG.M, XC);     % Partial COM's w.r.t. base
   
    for i = 1 : 7
        J(:,i) = W(i) * cross(Z(:,i), P(:,i) - O(:,i)); 
    end
    
end

function [ P ] = LegPCOM(M, COM)

    P = zeros(3,7); 
    
    for i = 1 : 7
        P(:,i) = CenterOfMass(M(i:end), COM(:,i:end)); 
    end

end

function [ W ] = LegWeights(M)
        
    Mtotal = sum(M); 
    W = zeros(7,1); 
    
    for i = 1 : length(M)
        W(i) = sum(M(i:end)) / Mtotal; 
    end

end

function [ COM ] = CenterOfMass(M, X)

    COM = zeros(3,1); 

    for i = 1 : length(M)
        COM = COM + (M(i) * X(:,i)); 
    end
    
    COM = COM * (1/sum(M)); 
end

function [ Jxy, Jz ] = SplitJcom(J)

	Jxy = J(1:2,:); 
    Jz  = J(3,:); 
    
end

% ------------------------------------------------------------------------
% END-EFFECTOR JACOBIANS
% ------------------------------------------------------------------------

function [ J ] = Jleg(SIDE, BIPED)

    J = zeros(6,20); 

    switch(SIDE)
        case 1 
            LEG = BIPED.L;
        case 2 
            LEG = BIPED.R; 
        otherwise
            error('Invalid side'); 
    end
    
    Z = BIPED.TW0(1:3,1:3) * LEG.Z;         %     Z = LEG.Z; 
    O = TransformArray(BIPED.TW0, LEG.O);   %     O = LEG.O; 
    EE = Transform(BIPED.TW0, LEG.XF);      %     EE = LEG.XF; 
    
    switch(SIDE)
        case 1 
            J(:,1:7) = JLi(Z, O, EE); 
        case 2 
            J(:,8:14) = JLi(Z, O, EE); 
        otherwise
            error('Invalid side'); 
    end
     
    J(:,15:20) = Jb(BIPED.TW0, LEG.XF); 
end

function [ J ] = JLi(Z, O, EE)

    J = zeros(6,7); 
    P = repmat(EE, 1, 7) - O;
    
    for i = 1 : 7
        J(:,i) = [cross(Z(:,i), P(:,i)); Z(:,i) ]; 
    end
    
end

function [ J ] = Jb(TW0, EE)
    
    J = eye(6); 
    
    [RB, PB] = Decompose(TW0); 
    P = Transform(TW0, EE) - PB; 
    J(1:3,4:6) = Rcross(RB, P); 
    
end

% ------------------------------------------------------------------------
% MATH FUNCTIONS
% ------------------------------------------------------------------------

function [ Ainv ] = Inverse( A )
    
    [m, n] = size(A);
    Ainv = zeros(n,m); 
    
    persistent k; 
    if isempty(k) k = 1e-3; end
    
    if (m == n)
        % Square Matrix 
        Ainv(:,:) = inv(A);
    else
        if k == 0 
            % METHOD 1: Pseudoinverse
            Ainv = pinv(A); 
        else
            % METHOD 2: Singularity Robust Inverse
            AAt = A * A'; 
            kI  = k * eye(size(AAt)); 
            Ainv(:,:) = A'/(AAt + kI); 
        end
    end
end

function [ S ] = Skew( a )
%SKEWSYMMETRIC Returns a 3x3 skew symmetric matrix for a 3x1 vector 'a'
    
    if length(a) ~= 3
        error('The input vector must be 3x1 or 1x3'); 
    end
    
    S = [... 
              0     -a(3)    a(2)   ;   ...
             a(3)     0     -a(1)   ;   ...
            -a(2)    a(1)      0    ;   ...
        ]; 

end

function [ N ] = Null(J)

    [~,n] = size(J); 
    N = eye(n,n) - (Inverse(J)*J); 

end

function [ P0 ] = Transform(T0W, PW)

    P = T0W * [PW; 1]; 
    P0 = P(1:3); 
    
end

function [ A0 ] = TransformArray(T0W, A)
    
    n = size(A,2); 
    P = T0W * [A; ones(1,n)]; 
    A0 = P(1:3,:); 

end

function [ X ] = Rcross(R, P)
    X = [cross(R(:,1), P) cross(R(:,2), P) cross(R(:,3), P)]; 
end

function [ R, P ] = Decompose(T)
    R = T(1:3, 1:3); 
    P = T(1:3, 4); 
end

% ////////////////////////////////////////////////////////////////////////
% ////////////////////////////////////////////////////////////////////////
