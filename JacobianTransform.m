function DQ = JacobianTransform(BIPED, DX)
%#codegen

    persistent S
    if isempty(S) S = [eye(14) zeros(14, 6)]; end
    
    Ju = [...
            Jleg(0, BIPED.L); ...
            Jleg(1, BIPED.R); ...
        ];
    
    DQ = S * Inverse(Ju) * DX;

end



% ------------------------------------------------------------------------
% END-EFFECTOR JACOBIANS
% ------------------------------------------------------------------------

function [ J ] = Jleg(SIDE, LEG)
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
    
    %J(:,15:20) = Jb(TW0, EE); 
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
    
    Xb = TW0(1:3,4);            % Floating base origin in world frame
    Xp = Transform(TW0, EE);    % End effector in world frame
    
    J(1:3,4) = cross(TW0(1:3,1), Xp-Xb); 
    J(1:3,5) = cross(TW0(1:3,2), Xp-Xb); 
    J(1:3,6) = cross(TW0(1:3,3), Xp-Xb); 
    
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

