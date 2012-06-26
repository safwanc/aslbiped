function DQ = JacobianTransform(BIPED, DX)
%#codegen

    persistent S
    if isempty(S) S = [eye(14) zeros(14, 6)]; end
    
    %% High Priority Tasks (Constraints + COM)
    Jcom_xyz = Jcom(BIPED); 
    
    [Jh, DXh] = Jconstraint(0, BIPED, Jcom_xyz, DX(1:3)); 

% 	Jl_xyzrpy = Jleg(1, BIPED.L, BIPED.TW0); 
% 	DX0 = [R0W*DX(1:3); R0W*DX(4:6); R0W*DX(7:9)]; 
%   [Jh, DXh] = Jconstraint(2, BIPED, [Jcom_xyz; Jl_xyzrpy], DX0); 
    
    DQ = S * Inverse(Jh) * DXh;

end

function [ Jc, DXc ] = Jconstraint(SIDE, BIPED, J, DX)

    % generate a jacobian with respect to both legs being fixed on the
    % ground. 
    
    switch(SIDE)
        case 1
            Js = Jleg(1, BIPED.L, BIPED.TW0); 
            DXs = [BIPED.L.FOOT.V; BIPED.L.FOOT.W]; 
        case 2 
            Js = Jleg(2, BIPED.R, BIPED.TW0); 
            DXs = [BIPED.R.FOOT.V; BIPED.R.FOOT.W]; 
        otherwise
            Js1 = Jleg(1, BIPED.L, BIPED.TW0);
            Js2 = Jleg(2, BIPED.R, BIPED.TW0); 
            
            % APPROACH 1: Position only
%           Js = [Js1(1:3,:); Js2(1:3,:)]; 
%           DXs = [BIPED.L.FOOT.V; BIPED.R.FOOT.V]; 
            
            % APPROACH 2: Position and Orientation
            Js = [Js1(1:6,:); Js2(1:6,:)]; 
            DXs = [...
                BIPED.L.FOOT.V; BIPED.L.FOOT.W; ...
                BIPED.R.FOOT.V; BIPED.R.FOOT.W; ...
                ]; 
    end

    if (rank(Js(:,15:20)) ~= 6)
        error('Constraint Jacobian Lost Rank'); 
    end

    DXs = zeros(size(Js,1),1);
    DXc = [DXs; DX];
	Jc  = [Js; J];

    
end

% ------------------------------------------------------------------------
% COM JACOBIANS
% ------------------------------------------------------------------------

function [ J ] = Jcom(BIPED)

    J = zeros(3,20); 
    RB = BIPED.TW0(1:3,1:3); 
    PB = BIPED.TW0(1:3,4); 

    % Leg Contributions
    J(:,1:7) = LegJCOM(BIPED.L);    % Left Leg
    J(:,8:14) = LegJCOM(BIPED.R);   % Right Leg
    
    % Base Contribution
    J(:,15:17) = eye(3); P = BIPED.COM - PB; 
    Z1 = RB(:,1); J(:,18) = cross(Z1, P);  
    Z2 = RB(:,2); J(:,19) = cross(Z2, P);  
    Z3 = RB(:,3); J(:,20) = cross(Z3, P);  
    
end

function [ J ] = LegJCOM(LEG)

    J = zeros(3,7); 
    
    persistent W
    
    if isempty(W)
        W = LegWeights(LEG.M);
    end

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

function [ J ] = Jtorso(BIPED)

    J = zeros(6,20); 
    EE = zeros(3,1); 
    
    J(:,1:7)    = JLi(BIPED.L.Z, BIPED.L.O, EE); 
    J(:,8:14)   = JLi(BIPED.R.Z, BIPED.R.O, EE); 
    J(:,15:20)  = Jb(BIPED.TW0, EE); 

end

function [ J ] = Jleg(SIDE, LEG, TW0)
    J = zeros(6,20); 
    
    Z = LEG.Z; 
    O = LEG.O; 
    EE = LEG.XF; 
    
    RB = TW0(1:3,1:3); 
    
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
    
%     J(1:3,1:14) = RB * J(1:3,1:14); 
%     J(4:6,1:14) = RB * J(4:6,1:14); 
    
    J(:,15:20) = Jb(TW0, EE); 
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
    Xb = TW0(1:3,4); Xp = Transform(TW0, EE); 
    
    z1 = TW0(1:3,1); 
    z2 = TW0(1:3,2); 
    z3 = TW0(1:3,3);
    p = Xp - Xb; 
    
    J(1:3,4) = cross(z1, p); 
    J(1:3,5) = cross(z2, p); 
    J(1:3,6) = cross(z3, p); 
    
end

% ------------------------------------------------------------------------
% PRIORITIZATION
% ------------------------------------------------------------------------

function [ DQ ] = PrioritizedTask(S, Jh, DXh, Jl, DXl)

    Nh = NullSpaceProjection(Jh); 
    
    DQ = S * (Inverse(Jh)*DXh + Nh*Inverse(Jl)*DXl); 

end

% ------------------------------------------------------------------------
% MATH FUNCTIONS
% ------------------------------------------------------------------------

function [ Ainv ] = Inverse( A )
    
    [m, n] = size(A); 
    
    persistent k; 
    if isempty(k) k = 1e-3; end
    
    if (m == n)
        % Square Matrix 
        Ainv = inv(A);
    else
        if k == 0 
            % METHOD 1: Pseudoinverse
            Ainv = pinv(A); 
        else
            % METHOD 2: Singularity Robust Inverse
            AAt = A * A'; 
            kI  = k * eye(size(AAt)); 
            Ainv = A'/(AAt + kI); 
        end
    end
end

function [ S ] = SkewSymmetric( a )
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

function [ N ] = NullSpaceProjection(J)

    [~,n] = size(J); 
    N = eye(n,n) - (Inverse(J) * J); 

end

function [ P0 ] = Transform(T0W, PW)

    P = T0W * [PW; 1]; 
    P0 = P(1:3); 
    
end

function [ J ] = Jv(Jvw)
    J = Jvw(1:3,:);     
end

function [ J ] = Jw(Jvw)
    J = Jvw(4:6,:); 
end

% ////////////////////////////////////////////////////////////////////////
% ////////////////////////////////////////////////////////////////////////

