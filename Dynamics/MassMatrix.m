function [A, b] = MassMatrix(x, dx, w)
%#codegen

    persistent LINK DOF
    
    if isempty(LINK)
        LINK = ASLBiped; 
    end

    if isempty(DOF)
        DOF = length(LINK) - 1; 
    end
    
    q = x(7:end); 
    dq = dx(7:end); 
    
    LINK(1).p = x(1:3);
    LINK(1).R = GetRotMatrix(x(4:6));
    LINK(1).vo = dx(1:3);
    LINK(1).w  = dx(4:6);

    for n = 1: DOF
        LINK(n+1).q = q(n); 
        LINK(n+1).dq = dq(n);
    end
    
    for j = 2 : length(LINK)
        MOM = LINK(j).mother;
        LINK(j).p = LINK(MOM).R * LINK(j).b + LINK(MOM).p;
        LINK(j).R = LINK(MOM).R * Rodriguez(LINK(j).a, LINK(j).q);
    end
    
    [A, b] = NewtonEulerDynamics(LINK, w);
    
end

%% Kinematics + Dynamics

function [A, b] = NewtonEulerDynamics(LINK, CONTACT)

	DOF = length(LINK) - 1 + 6;
    
	A = zeros(DOF,DOF);
    b = RNE(LINK, CONTACT, 0); 

    for n = 1 : DOF
        A(:,n) = RNE(LINK, CONTACT, n) - b; 
    end
    
end

function u = RNE(BIPED, CONTACT, j)
    LINK = BIPED; 
    
    % Contact Wrenches (Forces & Moments)
	fl = CONTACT(1:3,1); tl = CONTACT(4:6,1); 
    fr = CONTACT(1:3,2); tr = CONTACT(4:6,2); 
    
    LINK(1).dvo = [0 0 0]';
    LINK(1).dw  = [0 0 0]';
    
    if j >= 1 && j <= 3 
        LINK(1).dvo(j) = 1;
    elseif j >= 4 && j <= 6
        LINK(1).dw(j-3) = 1;
    end
    
    LINK(1).dvo = LINK(1).dvo + [0 0 9.81]';
    
    % Actuated DOF
    for n=1:length(LINK)-1
        if n == j - 6
            LINK(n+1).ddq = 1;
        else
            LINK(n+1).ddq = 0;
        end
    end
    
    % Forward Chain
    for i = 2 : length(LINK)
        LINK = FwdAllKinematics(LINK,i);
    end
    
    % Backward Chain
    nlink = length(LINK); 
    nleg = (nlink-1)/2; 
    ua = zeros(nleg*2,1);
    
	for i = (nleg+1) : -1: 2
        [fl, tl, ua(i-1)] = InvDynamics(LINK, i, fl, tl);
	end
    
    for i = nlink : -1: (nleg+2)
        [fr, tr, ua(i-1)] = InvDynamics(LINK, i, fr, tr);
    end

    [fb, tb, ~] = InvDynamics(LINK, 1, (fr+fl), (tr+tl));
    u = [fb', tb', ua']';
    
end

function [ LINK ] = FwdAllKinematics(LINK, j)

    MOM = LINK(j).mother;
    
    % position and orientation
    LINK(j).p = LINK(MOM).R * LINK(j).b + LINK(MOM).p;
    LINK(j).R = LINK(MOM).R * Rodriguez(LINK(j).a, LINK(j).q);

    % spatial velocity
    hw = LINK(MOM).R * LINK(j).a;  % axis vector in world frame
    hv = cross(LINK(j).p, hw);      % p_i x axis

    LINK(j).hw = hw;                % store h1 and h2 for future use
    LINK(j).hv = hv;

    LINK(j).w  = LINK(MOM).w  + hw * LINK(j).dq;
    LINK(j).vo = LINK(MOM).vo + hv * LINK(j).dq;

    % spatial acceleration
    dhv = cross(LINK(MOM).w, hv) + cross(LINK(MOM).vo, hw);
    dhw = cross(LINK(MOM).w, hw);

    LINK(j).dw  = LINK(MOM).dw  + dhw * LINK(j).dq + hw * LINK(j).ddq;
    LINK(j).dvo = LINK(MOM).dvo + dhv * LINK(j).dq + hv * LINK(j).ddq;

end

function [f, t, u] = InvDynamics(LINK, j, f0, t0)

    c = LINK(j).R * LINK(j).c + LINK(j).p;   % center of mass
    I = LINK(j).R * LINK(j).I * LINK(j).R';  % inertia in world frame
    
    c_hat = SkewSymmetric(c);
    I = I + LINK(j).m * c_hat * c_hat'; 

    P = LINK(j).m * (LINK(j).vo + cross(LINK(j).w,c));   % linear  momentum
    L = LINK(j).m * cross(c,LINK(j).vo) + I * LINK(j).w; % angular momentum

    f1 = LINK(j).m * (LINK(j).dvo + cross(LINK(j).dw,c))   + cross(LINK(j).w,P);
    t1 = LINK(j).m * cross(c,LINK(j).dvo) + I * LINK(j).dw + cross(LINK(j).vo,P) + cross(LINK(j).w,L);

    f = f0 + f1; % link force
    t = t0 + t1; % link moment
    
    u = LINK(j).hv' * f + LINK(j).hw' * t;  % joint driving force

end

%% UTILITY
function R = Rodriguez(w,dt)
% w should be column vector of size 3
    th = norm(w,2)*dt;
    wn = w/norm(w,2);		% normarized vector
    w_wedge = [0 -wn(3) wn(2);wn(3) 0 -wn(1);-wn(2) wn(1) 0];
    R = eye(3) + w_wedge * sin(th) + w_wedge^2 * (1-cos(th));

end

function [ R ] = GetRotMatrix(RPY)

    RRoll   = Rx(RPY(1)); 
    RPitch  = Ry(RPY(2)); 
    RYaw    = Rz(RPY(3)); 
    
    R = RYaw * RPitch * RRoll;

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

function c_hat = SkewSymmetric(c)
    c_hat = [0 -c(3) c(2);c(3) 0 -c(1);-c(2) c(1) 0];
end

%% DATA STORE

function [uLINK] = ASLBiped

    uLINK = [ ...
				struct('name', 'TORSO-001', 	'sister', 0,	'child', 2, 	'mother', 0, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'HIP-001-1', 	'sister', 9,	'child', 3, 	'mother', 1, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'HIP-002-1', 	'sister', 0,	'child', 4, 	'mother', 2, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'THIGH-001-1',	'sister', 0,	'child', 5, 	'mother', 3, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'KNEE-001-1',	'sister', 0,	'child', 6, 	'mother', 4, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'SHANK-001-1',	'sister', 0,	'child', 7, 	'mother', 5, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'FOOT-001-1',	'sister', 0,	'child', 8, 	'mother', 6, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'FOOT-002-1',	'sister', 0,	'child', 0, 	'mother', 7, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'HIP-001-2', 	'sister', 0,	'child', 10,	'mother', 1, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'HIP-002-2', 	'sister', 0,	'child', 11,	'mother', 9, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'THIGH-001-2',	'sister', 0,	'child', 12,	'mother', 10, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'KNEE-001-2',	'sister', 0,	'child', 13,	'mother', 11, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'SHANK-001-2',	'sister', 0,	'child', 14,	'mother', 12, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'FOOT-001-2',	'sister', 0,	'child', 15,	'mother', 13, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'FOOT-002-2',	'sister', 0,	'child', 0, 	'mother', 14, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1))  	...
			]; 
    
	% Kinematics
	uLINK(1).a	= [0 0 1]'; 
	uLINK(2).a	= [0 0 1]'; 
	uLINK(3).a	= [1 0 0]'; 
	uLINK(4).a	= [0 1 0]';
	uLINK(5).a	= [0 1 0]';
	uLINK(6).a	= [0 0 1]';
	uLINK(7).a	= [0 1 0]';
	uLINK(8).a	= [1 0 0]';
	uLINK(9).a	= [0 0 1]'; 
	uLINK(10).a	= [1 0 0]'; 
	uLINK(11).a	= [0 1 0]';
	uLINK(12).a	= [0 1 0]';
	uLINK(13).a	= [0 0 1]'; 
	uLINK(14).a	= [0 1 0]';
	uLINK(15).a	= [1 0 0]'; 
	
	uLINK(1).b 	= [0 0 0.742813]'; 
	uLINK(2).b 	= [0 -0.13 -0.011525]'; 
	uLINK(3).b 	= [-0.00067 0 -0.0432375]'; 
	uLINK(4).b 	= [0 0 0]'; 
	uLINK(5).b 	= [0 0 -0.2837625]'; 
	uLINK(6).b 	= [0.0031 0 -0.046525]'; 
	uLINK(7).b 	= [0 -0.0002375 -0.2927625]'; 
	uLINK(8).b 	= [0 0 -0.0002]'; 
	uLINK(9).b 	= [0 0.13 -0.011525]'; 
	uLINK(10).b	= [-0.00067 0 -0.0432375]'; 
	uLINK(11).b	= [0 0 0]'; 
	uLINK(12).b	= [0 0 -0.2837625]'; 
	uLINK(13).b	= [0.0031 0 -0.046525]'; 
	uLINK(14).b	= [0 -0.0002375 -0.2927625]'; 
	uLINK(15).b	= [0 0 -0.0002]'; 
    	
	% Mass Properties
	uLINK(1).m	= 4.4618048857868278; 
	uLINK(2).m	= 1.5610618990487068; 
	uLINK(3).m	= 0.78755487513250966; 
	uLINK(4).m	= 4.065003; 
	uLINK(5).m	= 0.60360793435450355;
	uLINK(6).m	= 3.939695; 
	uLINK(7).m	= 0.92900344137961666;
	uLINK(8).m	= 0.67770501948995632;
	uLINK(9).m	= 1.5610618990487068; 
	uLINK(10).m	= 0.78755487513250966; 
	uLINK(11).m	= 4.065003; 
	uLINK(12).m	= 0.60360793435450355;
	uLINK(13).m	= 3.939695;
	uLINK(14).m	= 0.92900344137961666;
	uLINK(15).m	= 0.67770501948995632;

	uLINK(1).I	=	[0.078175,0,-3.9252e-005;0,0.01298,0;-3.9252e-005,0,0.072286];
	uLINK(2).I	=	[0.0046105,0,-0.0016855;0,0.0078557,0;-0.0016855,0,0.0037689];
	uLINK(3).I	=	[0.0013056,7.0444e-005,0;7.0444e-005,0.0004486,0;0,0,0.0016328];
	uLINK(4).I	=	[0.031657,1.8967e-006,3.2522e-006;1.8967e-006,0.025913,0.0022484;3.2522e-006,0.0022484,0.0070284];
	uLINK(5).I	=	[0.0010577,0,-8.4362e-006;0,0.00054754,0;-8.4362e-006,0,0.00090994];
	uLINK(6).I	=	[0.031949,1.422e-006,-1.7738e-006;1.422e-006,0.025846,-0.0031217;-1.7738e-006,-0.0031217,0.0075815];
	uLINK(7).I	=	[0.0015381,-0.00012181,0;-0.00012181,0.00053903,0;0,0,0.0018796];
	uLINK(8).I	=	eye(3)*0.05;
	uLINK(9).I	=	[0.0046105,0,-0.0016855;0,0.0078557,0;-0.0016855,0,0.0037689];
	uLINK(10).I =	[0.0013056,7.0444e-005,0;7.0444e-005,0.0004486,0;0,0,0.0016328];
	uLINK(11).I =	[0.031657,1.8967e-006,3.2522e-006;1.8967e-006,0.025913,0.0022484;3.2522e-006,0.0022484,0.0070284];
	uLINK(12).I =	[0.0010577,0,-8.4362e-006;0,0.00054754,0;-8.4362e-006,0,0.00090994];
	uLINK(13).I =	[0.031949,1.422e-006,-1.7738e-006;1.422e-006,0.025846,-0.0031217;-1.7738e-006,-0.0031217,0.0075815];
	uLINK(14).I =	[0.0015381,-0.00012181,0;-0.00012181,0.00053903,0;0,0,0.0018796];
	uLINK(15).I =	eye(3)*0.05;

	uLINK(1).c	= [-6.492E-05 1.11E-06 0.04112821]'; 
	uLINK(2).c	= [-0.0706089 -2.7E-07 0.02662483]';
	uLINK(3).c	= [-0.00251109 0.03605746 -7.56E-06]';
	uLINK(4).c	= [-7.097E-05 0.00651026 -0.14876336]';
	uLINK(5).c	= [0.00280592 7.16E-06 -0.03987072]';
	uLINK(6).c	= [-8.052E-05 0.00736782 -0.14617992]';
	uLINK(7).c	= [0.00496159 0.02653546 -2E-08]';
	uLINK(8).c	= [0.02543338 0 -0.04207567]';
	uLINK(9).c	= [-0.0706089 -2.7E-07 0.02662483]'; 
	uLINK(10).c = [-0.00251109 -0.03605746 -7.56E-06]';  
	uLINK(11).c = [-7.097E-05 -0.00651026 -0.14876336]';
	uLINK(12).c = [0.00280592 7.16E-06 -0.03987072]';
	uLINK(13).c = [-8.052E-05 -0.00736782 -0.14617992]';
	uLINK(14).c = [0.00496159 -0.02653546 -2E-08]';
	uLINK(15).c = [0.02543338 0 -0.04207567]';
    	
    % Finalize
    uLINK(1).p = uLINK(1).b; 
    uLINK(1).R = eye(3); 
    
end