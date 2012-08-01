function [Jeff, Beff] = DrivetrainDynamics(q, dq)
%#codegen

    persistent LINK DOF MOTOR
    
    if isempty(LINK)
        LINK = ASLBiped; 
    end
    
    if isempty(MOTOR)
        MOTOR = ASLMotors; 
    end

    if isempty(DOF)
        DOF = length(LINK) - 1; 
    end
    
    LINK(1).p = zeros(3,1); %T01(1:3,4); 
    LINK(1).R = eye(3);     %T01(1:3,1:3);
    
    for n = 1: DOF
        LINK(n+1).q = q(n); 
        LINK(n+1).dq = dq(n); 
    end
    
    for j = 2 : length(LINK)
        MOM = LINK(j).mother;
        LINK(j).p = LINK(MOM).R * LINK(j).b + LINK(MOM).p;
        LINK(j).R = LINK(MOM).R * Rodriguez(LINK(j).a, LINK(j).q);
    end
    
    Aii = diag(MassMatrix(LINK));
    
    Jeff = Aii + MOTOR.J; 
    Beff = MOTOR.B; 
end

function A = MassMatrix(LINK)

	DOF = length(LINK) - 1; 
    
	A = zeros(DOF,DOF);
    b = RNEA(LINK, 0);
    
    for n = 1 : DOF
        A(:,n) = RNEA(LINK, n) - b; 
    end
    
end

function u = RNEA(LINK, j)

    u = zeros(length(LINK)-1,1); 
    fl = [0,0,0]'; tl = [0,0,0]'; 
    
    % Actuated DOF
    for n=1:length(LINK)-1
        if n == j
            LINK(n+1).ddq = 1;
        else
            LINK(n+1).ddq = 0;
        end
    end
           
    % Forward Chain
    for i = 2 : length(LINK)
        [LINK] = FwdAllKinematics(LINK,i);
    end
    
    % Backward Chain
	for i = length(LINK) : -1: 2
        [fl, tl, u(i-1)] = InvDynamics(LINK, i, fl, tl);
	end
    
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

function c_hat = SkewSymmetric(c)
    c_hat = [0 -c(3) c(2);c(3) 0 -c(1);-c(2) c(1) 0];
end

%% DATA STORE
function [LINK] = ASLBiped

    LINK = [ ...
				struct('name', 'TORSO-001', 	'sister', 0,	'child', 2, 	'mother', 0, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'HIP-001-1', 	'sister', 0,	'child', 3, 	'mother', 1, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'HIP-002-1', 	'sister', 0,	'child', 4, 	'mother', 2, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'THIGH-001-1',	'sister', 0,	'child', 5, 	'mother', 3, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'KNEE-001-1',	'sister', 0,	'child', 6, 	'mother', 4, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'SHANK-001-1',	'sister', 0,	'child', 7, 	'mother', 5, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'FOOT-001-1',	'sister', 0,	'child', 8, 	'mother', 6, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1)), 	...
				struct('name', 'FOOT-002-1',	'sister', 0,	'child', 0, 	'mother', 7, 	'q', 0, 	'dq', 0, 	'ddq', 0, 	'u', 0, 	'a', zeros(3,1), 	'b', zeros(3,1), 	'c', zeros(3,1), 	'm', 0, 	'I', zeros(3), 	'p', zeros(3,1), 	'R', eye(3), 	'Ir', 0, 	'gr', 0, 	'vo', zeros(3,1), 	'w', zeros(3,1), 	'dvo', zeros(3,1), 	'dw', zeros(3,1), 	'hw', zeros(3,1), 	'hv', zeros(3,1))  	...
            ]; 
    
	% Kinematics
	LINK(1).a	= [0 0 0]'; 
	LINK(2).a	= [0 0 1]'; 
	LINK(3).a	= [1 0 0]'; 
	LINK(4).a	= [0 1 0]';
	LINK(5).a	= [0 1 0]';
	LINK(6).a	= [0 0 1]';
	LINK(7).a	= [0 1 0]';
	LINK(8).a	= [1 0 0]';
	
	LINK(1).b 	= [0 0 0]'; 
	LINK(2).b 	= [0 0.13 -0.011525]'; 
	LINK(3).b 	= [-0.00067 0 -0.0432375]'; 
	LINK(4).b 	= [0 0 0]'; 
	LINK(5).b 	= [0 0 -0.2837625]'; 
	LINK(6).b 	= [0.0031 0 -0.046525]'; 
	LINK(7).b 	= [0 -0.0002375 -0.2927625]'; 
	LINK(8).b 	= [0 0 -0.0002]'; 
    	
	% Mass Properties
	LINK(1).m	= 0; %4.4618048857868278; 
	LINK(2).m	= 1.5610618990487068; 
	LINK(3).m	= 0.78755487513250966; 
	LINK(4).m	= 4.065003; 
	LINK(5).m	= 0.60360793435450355;
	LINK(6).m	= 3.939695; 
	LINK(7).m	= 0.92900344137961666;
	LINK(8).m	= 0.67770501948995632;

	LINK(1).I	=	eye(3)*0.05; %[0.078175,0,-3.9252e-005;0,0.01298,0;-3.9252e-005,0,0.072286];
	LINK(2).I	=	[0.0046105,0,-0.0016855;0,0.0078557,0;-0.0016855,0,0.0037689];
	LINK(3).I	=	[0.0013056,7.0444e-005,0;7.0444e-005,0.0004486,0;0,0,0.0016328];
	LINK(4).I	=	[0.031657,1.8967e-006,3.2522e-006;1.8967e-006,0.025913,0.0022484;3.2522e-006,0.0022484,0.0070284];
	LINK(5).I	=	[0.0010577,0,-8.4362e-006;0,0.00054754,0;-8.4362e-006,0,0.00090994];
	LINK(6).I	=	[0.031949,1.422e-006,-1.7738e-006;1.422e-006,0.025846,-0.0031217;-1.7738e-006,-0.0031217,0.0075815];
	LINK(7).I	=	[0.0015381,-0.00012181,0;-0.00012181,0.00053903,0;0,0,0.0018796];
	LINK(8).I	=	eye(3)*0.05;

	LINK(1).c	= zeros(3,1); %[-6.492E-05 1.11E-06 0.04112821]'; 
	LINK(2).c	= [-0.0706089 -2.7E-07 0.02662483]';
	LINK(3).c	= [-0.00251109 0.03605746 -7.56E-06]';
	LINK(4).c	= [-7.097E-05 0.00651026 -0.14876336]';
	LINK(5).c	= [0.00280592 7.16E-06 -0.03987072]';
	LINK(6).c	= [-8.052E-05 0.00736782 -0.14617992]';
	LINK(7).c	= [0.00496159 0.02653546 -2E-08]';
	LINK(8).c	= [0.02543338 0 -0.04207567]';
    
    % Finalize
    LINK(1).p = LINK(1).b; 
    LINK(1).R = eye(3); 
    
end

function [MOTOR] = ASLMotors

	% Motor Parameters
    [~, Km1, Kb1, R1, ~, Jm1, gr1] = Micromo3257
    [~, Km2, Kb2, R2, ~, Jm2, gr2] = Micromo3242
    
    J1 = Jm1 * (gr1^2);     B1 = (gr1^2) * (Kb1*Km1/R1); 
    J2 = Jm2 * (gr2^2);     B2 = (gr2^2) * (Kb2*Km2/R2); 
    
    MOTOR = struct(...
        'J', [J1 J1 J1 J1 J1 J1 J2]', ...
        'B', [B1 B1 B1 B1 B1 B1 B2]'  ...
        ); 

end

function [Kv, Km, Kb, R, L, Jm, gr] = Micromo3257

    Kv = 52.3598775;  %   Speed Constant        [(rad/s)/V]
    Km = 0.01910000;  %	  Torque Constant       [Nm/A]
    Kb = 0.01909859;  %   Back EMF Constant     [V/(rad/s)]
    R  = 0.41000000;  %   Armature Resistance   [Ohms]
    L  = 0.00007000;  %   Armature Inductance   [H]
    Jm = 0.00000420;  %   Motor Inertia         [kg m^2]
    gr = 240.000000;  %   Gear Ratio            [:1]
    
end

function [Kv, Km, Kb, R, L, Jm, gr] = Micromo3242

    Kv = 48.5899663;  %   Speed Constant        [(rad/s)/V]
    Km = 0.02060000;  %	  Torque Constant       [Nm/A]
    Kb = 0.02058038;  %   Back EMF Constant     [V/(rad/s)]
    R  = 1.27000000;  %   Armature Resistance   [Ohms]
    L  = 0.00013500;  %   Armature Inductance   [H]
    Jm = 0.00000250;  %   Motor Inertia         [kg m^2]
    gr = 68.0000000;  %   Gear Ratio            [:1]
    
end
