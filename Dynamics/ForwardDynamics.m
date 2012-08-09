function [ddx, ddq] = ForwardDynamics(u, A, b)
%#codegen

    ddx = A\(-b+[zeros(1,6) u']');
    ddq = ddx(7:end); 
   
end