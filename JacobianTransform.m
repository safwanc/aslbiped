function DQ = JacobianTransform(TW0, T0N, DX, n)
%#codegen

    persistent S
    if isempty(S) S = [eye(7) zeros(7, 6)]; end
 
    P0 = [0 0.13 -0.011525]'; 
    F0 = Transform(T0N(:,:,7), [0 0 -0.06480]'); 
    
    Jp = JLi(TW0, T0N, P0); 
    Jc = JLi(TW0, T0N, F0); 

    if (rank(Jc(:,8:13)) ~= 6)
        error('Constraint Jacobian lost full rank'); 
    end
    
    Rb = TW0(1:3,1:3); 
    Ju = [Jc; Jp(1:3,:)]; 
    DQ = S * Inverse(Ju) * [zeros(6,1); Rb*DX]; 

end

function [ DQ ] = PrioritizedTask(Jh, DXh, Jl, DXl)
    
    n = size(Jh, 2) - 6; 
    
    S = [eye(n) zeros(6,n)]; 
    Nh = NullSpaceProjection(Jh); 
    
    DQ = S * (Inverse(Jh)*DXh + Nh*Inverse(Jl)*DXl); 

end

% ------------------------------------------------------------------------
% END-EFFECTOR JACOBIANS
% ------------------------------------------------------------------------

function [ J ] = JLi(TW0, T0N, EE)

    J = zeros(6,13); 

    Z = LegAxis(T0N);              % joint axes w.r.t. base frame
    O = squeeze(T0N(1:3,4,:));  % joint origins w.r.t. base frame
    
    if (size(EE,2) == 1)
        P = repmat(EE, 1, 7) - O;   % vector from joint origin to end effector
    else
        p = EE - 0; 
    end
    
    for i = 1 : 7
        J(:,i) = [cross(Z(:,i), P(:,i)); Z(:,i) ]; 
    end
    
    J(:,8:13) = Jb(TW0, EE); 
    
end

function [ J ] = Jb(TW0, EE)
    J = eye(6); 
    Rb = TW0(1:3,1:3); Xb = TW0(1:3,4); 
    Xp = Transform(TW0, EE); 
    J(1:3,4) = cross(Rb(1:3,1), Xp-Xb); 
    J(1:3,5) = cross(Rb(1:3,2), Xp-Xb); 
    J(1:3,6) = cross(Rb(1:3,3), Xp-Xb); 
end

% ------------------------------------------------------------------------
% UTILITY FUNCTIONS
% ------------------------------------------------------------------------


function [ Z ] = LegAxis(T0N)
    Z = zeros(3,7); 
    Z(:,1) = T0N(1:3, 3, 1); 
	Z(:,2) = T0N(1:3, 1, 2); 
	Z(:,3) = T0N(1:3, 2, 3); 
	Z(:,4) = T0N(1:3, 2, 4); 
	Z(:,5) = T0N(1:3, 3, 5); 
	Z(:,6) = T0N(1:3, 2, 6); 
	Z(:,7) = T0N(1:3, 1, 7); 
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

