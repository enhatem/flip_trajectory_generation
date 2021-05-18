clear; 
close all; 
clc;

% objective function
fun = @objective_function;

% constraints
nonlcon = @NL_bounds;

lb = [ 3   -2 -2 -2 0    3/2*pi   0.01   0.01   0.01  ];
ub = [ 3.5  2  2  2 pi/2   2*pi   inf inf inf];

%%
rng(200)
options = optimoptions('ga','ConstraintTolerance',1e-6,'PlotFcn', @gaplotbestf);
x = ga(fun,9,[],[],[],[],lb,ub,nonlcon)