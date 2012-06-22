function BIPED   = ForwardKinematics( X )
%#codegen
    
	TW0 = reshape(X(1:16), 4, 4);  

    persistent T0L T0R
    if isempty(T0L) T0L = [eye(3) [0 -0.13 -0.011525]'; 0 0 0 1]; end
    if isempty(T0R) T0R = [eye(3) [0  0.13 -0.011525]'; 0 0 0 1]; end
    
    TL = FKleg(T0L, X(17:23)); 
    TR = FKleg(T0R, X(24:30)); 
    
    BIPED = struct('TW', TW0, 'TL', TL, 'TR', TR); 

end

function [ T ] = FKleg( T0, Q )
    T = zeros(4,4,7); 
    T1 = T0 * [Rz(Q(1)) [0 0 0]'; 0 0 0 1]; 
    T2 = T1 * [Rx(Q(2)) [-0.00067 0 -0.0432375]'; 0 0 0 1]; 
    T3 = T2 * [Ry(Q(3)) [0 0 0]'; 0 0 0 1]; 
    T4 = T3 * [Ry(Q(4)) [0 0 -0.2837625]'; 0 0 0 1]; 
    T5 = T4 * [Rz(Q(5)) [0.0031 0 -0.046525]'; 0 0 0 1]; 
    T6 = T5 * [Ry(Q(6)) [0 -0.0002375 -0.2927625]'; 0 0 0 1]; 
    T7 = T6 * [Rx(Q(7)) [0 0 -0.0002]'; 0 0 0 1]; 
    
	T(:,:,1) = T1;
	T(:,:,2) = T2;
	T(:,:,3) = T3;
	T(:,:,4) = T4;
	T(:,:,5) = T5;
	T(:,:,6) = T6;
	T(:,:,7) = T7;
    
end

function [ TS, TS1, TS2, TS3, TS4 ] = FKankle( TA )

    TS   = TA * [eye(3) [0 0 -0.06480]'; 0 0 0 1];
    TS1  = TS * [eye(3) [-0.050922 0.035 0]'; 0 0 0 1];   % w.r.t. sole
    TS2  = TS * [eye(3) [-0.050922 -0.035 0]'; 0 0 0 1];  % w.r.t. sole
    TS3  = TS * [eye(3) [0.119078 -0.035 0]'; 0 0 0 1];   % w.r.t. sole
    TS4  = TS * [eye(3) [0.119078 0.035 0]'; 0 0 0 1];    % w.r.t. sole
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