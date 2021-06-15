function [A,b] = build_matrix(start,goal,T)

   b = [start goal]';

   A = [1 0 0     0     0       0; 
        0 1 0     0     0       0;
        0 0 2     0     0       0;
        1 T T^2   T^3   T^4     T^5; 
        0 1 2*T   3*T^2 4*T^3   5*T^4;  
        0 0 2     6*T   12*T^2  20*T^3];

end