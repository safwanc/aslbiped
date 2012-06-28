function [ TW1, TW2, TW3, TW4, TW5, TW6, TW7 ] = SceneKinematics( TW0, T0 )
%codegen

	TW1  = TW0 * T0(:,:,1); 
    TW2  = TW0 * T0(:,:,2);
    TW3  = TW0 * T0(:,:,3);
    TW4  = TW0 * T0(:,:,4);
    TW5  = TW0 * T0(:,:,5);
    TW6  = TW0 * T0(:,:,6);
    TW7	 = TW0 * T0(:,:,7);
    
end