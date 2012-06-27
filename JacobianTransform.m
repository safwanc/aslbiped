function [DQ, RESET] = JacobianTransform(MODE, BIPED, DX)
%#codegen

    DQ = zeros(14,1); 
    RESET = 0; 

    % Controller State Variables
    persistent LAST
    if isempty(LAST)
        LAST = struct( ...
            'MODE', MODE ...
            ); 
    end

    % Constant Variables (Optimization)
    persistent L R LR S JB I
    if isempty(L)   L = 1; end
    if isempty(R)   R = 2; end
    if isempty(LR) LR = 0; end
    if isempty(I)   I = eye(20); end
    if isempty(S)   S = [eye(14) zeros(14, 6)]; end
    if isempty(JB) JB = [zeros(6,14) eye(6)];   end
    
    if MODE ~= 0
        [ Jcom_xy, Jcom_z ] = SplitJcom(Jcom(BIPED)); 
        DXcom_xy = DX(1:2); DXcom_z = DX(3); 

        Jbase_rpy = JB(4:6,:); 
        DXbase_rpy = DX(4:6); 

        Jswing = Jleg(L, BIPED.L, BIPED.TW0);
        DXswing = DX(7:12); 

        %% High Priority Tasks
        Jhigh   = [Jcom_xy; Jswing];
        DXhigh  = [DXcom_xy; DXswing];  

        [Jh, DXh] = Jconstraint(R, BIPED, Jhigh, DXhigh); 

        %% Low Priority Tasks
        Jl  = Jcom_z; 
        DXl = DXcom_z; 

        %% Prioritization Framework
        DQ(:,:) = S * Prioritized(Jh, DXh, Jl, DXl); 
    end
    
    if MODE ~= LAST.MODE
        RESET = 1; 
    end
    
    LAST.MODE = MODE; 
end

function [ DQ ] = Prioritized(J1, X1, J2, X2)

    DQ = Inverse(J1)*X1 + (Inverse(J2*Null(J1))*(X2-(J2*Inverse(J1)*X1))); 

end

function [ Jc, DXc ] = Jconstraint(SIDE, BIPED, J, DX)
    
    DXs = zeros(6,1);
    
     switch(SIDE)
         
        case 1  % Left Foot Single Support 
            Js = Jleg(1, BIPED.L, BIPED.TW0); 
            if ~InContact(BIPED.R.FOOT)
                DXs = [BIPED.L.FOOT.V; BIPED.L.FOOT.W]; 
            end
            
        case 2 % Right Foot Single Support 
            Js = Jleg(2, BIPED.R, BIPED.TW0); 
            if ~InContact(BIPED.L.FOOT)
                DXs = [BIPED.R.FOOT.V; BIPED.R.FOOT.W]; 
            end
            
         otherwise
             error('Unsupported'); 
            
            %DXs = [BIPED.R.FOOT.V; BIPED.R.FOOT.W]; 
%         otherwise % Double Support  
%             Js1 = Jleg(1, BIPED.L, BIPED.TW0);
%             Js2 = Jleg(2, BIPED.R, BIPED.TW0);
%             Js = [Js1(1:6,:); Js2(1:6,:)]; 
%             DXs = [...
%                 BIPED.L.FOOT.V; BIPED.L.FOOT.W; ...
%                 BIPED.R.FOOT.V; BIPED.R.FOOT.W; ...
%                 ]; 
%             DXs = [...
%                 BIPED.L.FOOT.V; zeros(3,1); ...
%                 BIPED.R.FOOT.V; zeros(3,1); ...
%                 ]; 
%             
            % DXs = zeros(size(Js,1),1);
            
     end
    
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

function [ J ] = Jleg(SIDE, LEG, TW0)
    J = zeros(6,20); 
    RB = TW0(1:3,1:3);
    Z = RB * LEG.Z; %LEG.Z; 
    O = TransformArray(TW0, LEG.O); %LEG.O; 
    EE = Transform(TW0, LEG.XF); 

%     Z = LEG.Z; 
%     O = LEG.O; 
%     EE = LEG.XF; 
    
    switch(SIDE)
        case 1 
            % Left Leg
            J(:,1:7) = JLi(Z, O, EE); 
        case 2 
            % Right Leg
            J(:,8:14) = JLi(Z, O, EE); 
        otherwise
            error('Invalid side'); 
    end
     
    J(:,15:20) = Jb(TW0, LEG.XF); 
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

function [ J ] = Jv(Jvw)
    J = Jvw(1:3,:);     
end

function [ J ] = Jw(Jvw)
    J = Jvw(4:6,:); 
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

