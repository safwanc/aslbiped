function DQ = JacobianTransform(BIPED, DX)
%#codegen

    persistent S
    if isempty(S) S = [eye(14) zeros(14, 6)]; end
    
    RB = BIPED.TW0(1:3,1:3); 
    COM0 = Transform(inv(BIPED.TW0), BIPED.COM); 
    
    J1 = Jcom(BIPED, RB, COM0); 
%     J2 = Jleg(0, BIPED.L, RB);
%     J3 = Jleg(1, BIPED.R, RB);
%     
%     Ju = [...
%             J1; ...
%             J2(1:3,:); ...
%             J3(1:3,:)
%         ];
    Ju = J1; 
    DQ = S * Inverse(Ju) * DX;

end

% ------------------------------------------------------------------------
% COM JACOBIANS
% ------------------------------------------------------------------------

function [ J ] = Jcom(BIPED, RB, COM)

    J = zeros(3,20); 
    J(:,1:7) = LegJCOM(BIPED.L); 
    J(:,8:14) = LegJCOM(BIPED.R); 
    J(:,15:17) = eye(3); 
    J(:,18) = cross(RB(:,1), COM); 
    J(:,19) = cross(RB(:,2), COM); 
    J(:,20) = cross(RB(:,3), COM); 
end

function [ J ] = LegJCOM(LEG)

    J = zeros(3,7); 
    
    persistent W
    
    if isempty(W)
        W = LegWeights(LEG.M);
    end

	Z = LEG.Z;              % Joint axis w.r.t base
    O = LEG.O;              % Joint origins w.r.t. base
    XC = LEG.XC;            % COM position w.r.t. base
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

% ------------------------------------------------------------------------
% END-EFFECTOR JACOBIANS
% ------------------------------------------------------------------------

function [ J ] = Jleg(SIDE, LEG, RB)
    J = zeros(6,20); 
    
    Z = LEG.Z; 
    O = LEG.O; 
    EE = LEG.XF; 
    
    switch(SIDE)
        case 0 
            % Left Leg
            J(:,1:7) = JLi(Z, O, EE); 
        case 1 
            % Right Leg
            J(:,8:14) = JLi(Z, O, EE); 
        otherwise
            error('Invalid side'); 
    end
    
    %J(:,15:20) = Jb(RB, (EE-LEG.T0N(1:3,4,1))); 
end

function [ J ] = JLi(Z, O, EE)

    J = zeros(6,7); 
    P = repmat(EE, 1, 7) - O;
    
    for i = 1 : 7
        J(:,i) = [cross(Z(:,i), P(:,i)); Z(:,i) ]; 
    end
    
end

function [ J ] = Jb(RB, EE)
    
    J = eye(6); 
    J(1:3,4) = cross(RB(1:3,1), EE); 
    J(1:3,5) = cross(RB(1:3,2), EE); 
    J(1:3,6) = cross(RB(1:3,3), EE); 
    
end

% ------------------------------------------------------------------------
% PRIORITIZATION
% ------------------------------------------------------------------------

function [ DQ ] = PrioritizedTask(Jh, DXh, Jl, DXl)
    
    n = size(Jh, 2) - 6; 
    
    S = [eye(n) zeros(6,n)]; 
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

% ////////////////////////////////////////////////////////////////////////
% ////////////////////////////////////////////////////////////////////////

