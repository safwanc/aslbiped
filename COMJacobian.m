function [ Jcom ] = COMJacobian(TW0, T0N)
    
    persistent Mb COMb
    
    if isempty(COMb)
        COMb = [-6.492E-05 1.11E-06 0.04112821]'; 
    end
    
    if isempty(Mb)
        Mb = 4.4618048857868278; 
    end

    [Jleg, XC, M] = LegJCOM(T0N); 
	RobotCOM = CenterOfMass([M; Mb], [XC COMb]); 
    
    Jcom(:,1:7) = Jleg; 
    Jcom(:,8:13) = Jb(TW0, RobotCOM);
    
end

%% Leg Specific Functions
function [ J, XC, Ml ] = LegJCOM(T0N)

    J = zeros(3,7); 
    
    persistent M W

    if isempty(M)
        M = LegMasses;
    end
    
    if isempty(W)
        W = LegWeights(M);
    end

    XC = LegCOM(T0N);           % COM position w.r.t. base
    P = LegPCOM(M, XC);         % Partial COM's w.r.t. base
    Z = LegAxis(T0N);           % Joint axis w.r.t base
    O = squeeze(T0N(1:3,4,:));	% Joint origins w.r.t. base
   
    for i = 1 : 7
        J(:,i) = W(i) * cross(Z(:,i), P(:,i) - O(:,i)); 
    end
    
    Ml = M; 
    
end


function [ P ] = LegPCOM(M, COM)

    P = zeros(3,7); 
    Mtotal = sum(M); 
    
    for i = 1 : 7
        P(:,i) = CenterOfMass(M(i:end), COM(:,i:end)) / Mtotal; 
    end

end

function [ XC ] = LegCOM(T0N)

    persistent LocalCOM
    if isempty(LocalCOM)
        LocalCOM = zeros(3,7); 
        LocalCOM(:,1) = [-0.0706089 -2.7E-07 0.02662483]';
        LocalCOM(:,2) = [-0.00251109 0.03605746 -7.56E-06]';
        LocalCOM(:,3) = [-7.097E-05 0.00651026 -0.14876336]';
        LocalCOM(:,4) = [0.00280592 7.16E-06 -0.03987072]';
        LocalCOM(:,5) = [-8.052E-05 0.00736782 -0.14617992]';
        LocalCOM(:,6) = [0.00496159 0.02653546 -2E-08]';
        LocalCOM(:,7) = [0.02543338 0 -0.04207567]';
    end
    
    XC = zeros(3,7); 
    
    for i = 1 : 7
        XC(:,i) = Transform(T0N(:,:,i), LocalCOM(:,i)); 
    end

end

function [ M ] = LegMasses

    M = zeros(7,1); 
    M(1) = 1.5610618990487068; 
    M(2) = 0.78755487513250966; 
    M(3) = 4.065003; 
    M(4) = 0.60360793435450355;
    M(5) = 3.939695; 
    M(6) = 0.92900344137961666;
    M(7) = 0.67770501948995632;

end

function [ W ] = LegWeights(M)
        
    Mtotal = sum(M); 
    W = zeros(7,1); 
    
    for i = 1 : length(M)
        W(i) = sum(M(i:end)) / Mtotal; 
    end

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

%% Utility Functions
function [ COM ] = CenterOfMass(M, X)

    COM = zeros(3,1); 

    for i = 1 : length(M)
        COM = COM + (M(i) * X(:,i)); 
    end
    
    COM = COM * (1/sum(M)); 
end