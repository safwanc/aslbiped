function BIPED = ForwardKinematics( X )
%#codegen
    
    persistent LEFT RIGHT
    if isempty(LEFT) LEFT   = 0; end
    if isempty(RIGHT) RIGHT = 1; end

	TW0 = reshape(X(1:16), 4, 4);  
       
    L = BipedLeg(LEFT, X(17:23)); 
    R = BipedLeg(RIGHT, X(24:30)); 
    B = BipedTorso; 
        
    M = [B.M L.M R.M]; 
    X = [B.XC L.XC R.XC]; 
    
    COM0 = CenterOfMass(M, X) 
    COMW = Transform(TW0, COM0); 
    
    BIPED = struct(...
        'TW0', TW0, 'COM', COMW, ...
        'B', B, 'L', L, 'R', R); 

end

%% Data Structures

function [ TOR ] = BipedTorso
    
    persistent Mb COMb
    
    if isempty(Mb)
        Mb = 4.4618048857868278; 
    end
    
    if isempty(COMb)
        COMb = [-6.492E-05 1.11E-06 0.04112821]'; 
    end
    
    TOR = struct('M', Mb, 'XC', COMb); 
    
end

function [ LEG ] = BipedLeg(SIDE, Q)
	
    persistent M
    
    if isempty(M) M = LegMasses; end
	
    T0 = eye(4); 
    COM = zeros(3,7); 

    switch(SIDE)
        case 0
            T0(1:3,4) = [0 -0.13 -0.011525]';
            COM(:,1)  = [-0.0706089 -2.7E-07 0.02662483]';
            COM(:,2)  = [-0.00251109 0.03605746 -7.56E-06]';
            COM(:,3)  = [-7.097E-05 0.00651026 -0.14876336]';
            COM(:,4)  = [0.00280592 7.16E-06 -0.03987072]';
            COM(:,5)  = [-8.052E-05 0.00736782 -0.14617992]';
            COM(:,6)  = [0.00496159 0.02653546 -2E-08]';
            COM(:,7)  = [0.02543338 0 -0.04207567]';
        case 1
            T0(1:3,4) = [0 0.13 -0.011525]';
            COM(:,1)  = [-0.0706089 -2.7E-07 0.02662483]';
            COM(:,2)  = [-0.00251109 -0.03605746 -7.56E-06]';
            COM(:,3)  = [-7.097E-05 -0.00651026 -0.14876336]';
            COM(:,4)  = [0.00280592 7.16E-06 -0.03987072]';
            COM(:,5)  = [-8.052E-05 -0.00736782 -0.14617992]';
            COM(:,6)  = [0.00496159 -0.02653546 -2E-08]';
            COM(:,7)  = [0.02543338 0 -0.04207567]';
        otherwise
            error('Unsupported side index'); 
    end
        
    T0N = LegTransforms(T0,Q); 
	XC  = LegCOM(T0N, COM);
    Z   = LegAxis(T0N); 
    O   = squeeze(T0N(1:3,4,:)); 
    [ XF, CP ] = FKankle(T0N(:,:,end));
    
    LEG = struct(...
        'T0N', T0N, ...
        'O', O,     ...
        'Z', Z,     ...
        'M', M,     ...
        'XC', XC,   ...
        'XF', XF,   ...
        'CP', CP    ...
        ); 
end


%% Leg Functions

function [ T0N ] = LegTransforms( T0, Q )

    T0N = zeros(4,4,7); 
    T1 = T0 * [Rz(Q(1)) [0 0 0]'; 0 0 0 1]; 
    T2 = T1 * [Rx(Q(2)) [-0.00067 0 -0.0432375]'; 0 0 0 1]; 
    T3 = T2 * [Ry(Q(3)) [0 0 0]'; 0 0 0 1]; 
    T4 = T3 * [Ry(Q(4)) [0 0 -0.2837625]'; 0 0 0 1]; 
    T5 = T4 * [Rz(Q(5)) [0.0031 0 -0.046525]'; 0 0 0 1]; 
    T6 = T5 * [Ry(Q(6)) [0 -0.0002375 -0.2927625]'; 0 0 0 1]; 
    T7 = T6 * [Rx(Q(7)) [0 0 -0.0002]'; 0 0 0 1]; 
    
	T0N(:,:,1) = T1;
	T0N(:,:,2) = T2;
	T0N(:,:,3) = T3;
	T0N(:,:,4) = T4;
	T0N(:,:,5) = T5;
	T0N(:,:,6) = T6;
	T0N(:,:,7) = T7;
    
end

function [ XF, CP ] = FKankle( TA )

    TS   = TA * [eye(3) [0 0 -0.06480]'; 0 0 0 1];
    TS1  = TS * [eye(3) [-0.050922 0.035 0]'; 0 0 0 1];   % w.r.t. sole
    TS2  = TS * [eye(3) [-0.050922 -0.035 0]'; 0 0 0 1];  % w.r.t. sole
    TS3  = TS * [eye(3) [0.119078 -0.035 0]'; 0 0 0 1];   % w.r.t. sole
    TS4  = TS * [eye(3) [0.119078 0.035 0]'; 0 0 0 1];    % w.r.t. sole
    
    XF = TS(1:3, 4); 
    CP = [TS1(1:3,4) TS2(1:3,4) TS3(1:3,4) TS4(1:3,4)]; 
end

function [ M ] = LegMasses

    M = zeros(1,7); 
    M(1) = 1.5610618990487068; 
    M(2) = 0.78755487513250966; 
    M(3) = 4.065003; 
    M(4) = 0.60360793435450355;
    M(5) = 3.939695; 
    M(6) = 0.92900344137961666;
    M(7) = 0.67770501948995632;

end

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


function [ XC ] = LegCOM(T0N, COM)

    XC = zeros(3,7); 
    
    for i = 1 : 7
        XC(:,i) = Transform(T0N(:,:,i), COM(:,i)); 
    end

end

%% Utility Functions
function [ COM ] = CenterOfMass(M, X)

    COM = zeros(3,1); 

    for i = 1 : length(M)
        COM = COM + (M(i) * X(:,i)); 
    end
    
    COM = COM * (1/sum(M)); 
end

function [ P0 ] = Transform(T0W, PW)

    P = T0W * [PW; 1]; 
    P0 = P(1:3); 
    
end

function [ R ] = Rx(q)
    
    cq = cos(q); 
    sq = sin(q); 

    R = [
        1   0    0
        0   cq  -sq
        0   sq   cq
        ];

end

function [ R ] = Ry(q)
    
    cq = cos(q); 
    sq = sin(q); 

    R = [
        cq  0   sq
        0   1   0
       -sq  0   cq
       ];

end

function [ R ] = Rz(q)
    
    cq = cos(q); 
    sq = sin(q); 

    R = [
        cq  -sq  0
        sq   cq  0
        0    0   1
        ];

end