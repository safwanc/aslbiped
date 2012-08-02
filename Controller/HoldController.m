function qref = HoldController(q)
%#codegen

    persistent qhold
    
    if isempty(qhold)
        qhold = q; 
    end
    
    qref = qhold; 
end