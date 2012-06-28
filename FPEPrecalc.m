function [ Wavg, Icom ] = FPEPrecalc( WB, DQ, COM, T0X, CGX )

    persistent I 
    if isempty(I) I = GetConstant('I'); end
    
    Rw = GetRotationMatrices(T0X); 
    Iw = zeros(3, 3, 15); 
    
    for i = 1 : 15
        % rotate tensors to world frame
        Iw(:,:,i) = Rw(:,:,i) * I(:,:,i); 
    end
    
    Iyy = squeeze(Iw(2,2,:));
    Icom = CalculateIcom(COM, CGX, Iyy); 

    W = GetAngularVelocities(DQ);
    for i = 1 : 14
        W(:,i) = Rw(:,:,i+1) * W(:,i); 
    end
    Wy = zeros(15,1);
    Wy(1) = WB(2); % Base angular velocity (pitch component)
    Wy(2:15) = squeeze(W(2,:)); 
    
    Wavg = CalculateWavg(Iyy, Wy); 

end

function [ Wavg ] = CalculateWavg(Iyy, Wy)

    W = 0; 
    
    for i = 1 : 15
        W = W + (Iyy(i)*Wy(i)); 
    end
    
    Wavg = W / sum(W); 

end

function [ Icom ] = CalculateIcom(COM, CGX, Iyy)

    I = zeros(15,1); 
    
    persistent m 
    if isempty(m) m = GetConstant('m'); end

    D = reshape(CGX, 3, 15) - repmat(COM, 1, 15); 
    
    for k = 1 : 15
        d = hypot(D(1,k), D(3,k));       % distance in XZ plane
        I(k) = Iyy(k) + (m(k)*(d^2));    % parallel axis theorem
    end
    
    Icom = sum(I); 
end

function [ R ] = GetRotationMatrices(T0X)
    % R = zeros(3,3,15); 
    T = zeros(4,4,15); 
    
    TColumns = reshape(T0X, 16, 15); 
    
    for i = 1 : 15
        T(:,:,i) = reshape(TColumns, 4, 4); 
    end
    
    R = squeeze(T(1:3,1:3,:)); 

end

function [ W ] = GetAngularVelocities(DQ)
    
    W = zeros(3,14); 
    W(3,1) = DQ(1); W(3,8)  = DQ(8); 
    W(1,2) = DQ(2); W(1,9)  = DQ(9); 
    W(2,3) = DQ(3); W(2,10) = DQ(10); 
    W(2,4) = DQ(4); W(2,11) = DQ(11); 
    W(3,5) = DQ(5); W(3,12) = DQ(12); 
    W(2,6) = DQ(6); W(2,13) = DQ(13); 
    W(1,7) = DQ(7); W(1,14) = DQ(14); 
    

end

function [ k ] = GetConstant(n)
    switch(n)
        case 'I'
            
            k = zeros(3, 3, 15); 
            k(:,:,1)  =	[0.078175,0,-3.9252e-005;0,0.01298,0;-3.9252e-005,0,0.072286];
            k(:,:,2)  =	[0.0046105,0,-0.0016855;0,0.0078557,0;-0.0016855,0,0.0037689];
            k(:,:,3)  =	[0.0013056,7.0444e-005,0;7.0444e-005,0.0004486,0;0,0,0.0016328];
            k(:,:,4)  =	[0.031657,1.8967e-006,3.2522e-006;1.8967e-006,0.025913,0.0022484;3.2522e-006,0.0022484,0.0070284];
            k(:,:,5)  =	[0.0010577,0,-8.4362e-006;0,0.00054754,0;-8.4362e-006,0,0.00090994];
            k(:,:,6)  =	[0.031949,1.422e-006,-1.7738e-006;1.422e-006,0.025846,-0.0031217;-1.7738e-006,-0.0031217,0.0075815];
            k(:,:,7)  =	[0.0015381,-0.00012181,0;-0.00012181,0.00053903,0;0,0,0.0018796];
            k(:,:,8)  =	eye(3)*0.05;
            k(:,:,9)  =	[0.0046105,0,-0.0016855;0,0.0078557,0;-0.0016855,0,0.0037689];
            k(:,:,10) =	[0.0013056,7.0444e-005,0;7.0444e-005,0.0004486,0;0,0,0.0016328];
            k(:,:,11) =	[0.031657,1.8967e-006,3.2522e-006;1.8967e-006,0.025913,0.0022484;3.2522e-006,0.0022484,0.0070284];
            k(:,:,12) =	[0.0010577,0,-8.4362e-006;0,0.00054754,0;-8.4362e-006,0,0.00090994];
            k(:,:,13) =	[0.031949,1.422e-006,-1.7738e-006;1.422e-006,0.025846,-0.0031217;-1.7738e-006,-0.0031217,0.0075815];
            k(:,:,14) =	[0.0015381,-0.00012181,0;-0.00012181,0.00053903,0;0,0,0.0018796];
            k(:,:,15) =	eye(3)*0.05;

        case 'm'
            
            k = zeros(15,1); 
            k(1)  = 4.4618048857868278; 
            k(2)  = 1.5610618990487068; 
            k(3)  = 0.78755487513250966; 
            k(4)  = 4.065003; 
            k(5)  = 0.60360793435450355;
            k(6)  = 3.939695; 
            k(7)  = 0.92900344137961666;
            k(8)  = 0.67770501948995632;
            k(9)  = 1.5610618990487068; 
            k(10) = 0.78755487513250966; 
            k(11) = 4.065003; 
            k(12) = 0.60360793435450355;
            k(13) = 3.939695;
            k(14) = 0.92900344137961666;
            k(15) = 0.67770501948995632;            
            
        otherwise
            error('Unknown constant value'); 
    end
end
