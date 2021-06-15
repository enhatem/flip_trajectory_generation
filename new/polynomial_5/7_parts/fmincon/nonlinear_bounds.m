function [c,ceq] = nonlinear_bounds(x)
%nonlinear_bounds extracts the nonlinear bounds that will be used in the
%optimization problem.

    global g t_step m u1_max 

    ceq = []; % no equality constraints
    
    z1      = x(1);
    z2      = x(2);
    z3      = x(3);
    z4      = x(4);
    z5      = x(5);
    z6      = x(6);
    z7      = x(7);
    z8      = x(8);
    z2d     = x(9);
    z3d     = x(10);
    z6d     = x(11);
    z7d     = x(12);
    z2dd    = x(13);
    z3dd    = x(14);
    z6dd    = x(15);
    z7dd    = x(16);
    phi2    = x(17);
    phi3    = x(18);
    phi4    = x(19);
    phi5    = x(20);
    phi6    = x(21);
    phi7    = x(22);
    phi2d   = x(23);
    phi3d   = x(24);
    phi6d   = x(25);
    phi7d   = x(26);
    phi2dd  = x(27);
    phi3dd  = x(28);
    phi6dd  = x(29);
    phi7dd  = x(30);
    t1      = x(31);
    t2      = x(32);
    t3      = x(33);
    t4      = x(34);
    t5      = x(35);
    t6      = x(36);
    t7      = x(37);

    
    
    
    
    % Bounds
    c = [ -min(u1);           % lower bound on u1 (u1>=0)
           max(u1)-0.24 ];    % upper bound on u1 (u1<=u1 max)
    
end

