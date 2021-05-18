clc 
clear all
close all

% Input parameters

nvars = 2; % number of variables
lb = [-5.12 -5.12]; % lower bound
ub = [ 5.12, 5.12]; % upper bound
PopulationSize_Data = 100; 
CrossoverFraction_Data = 0.7;
MaxGenerations_Data = 100;
MaxStallGenerations_Data = 100;
InitialPopulationMatrix_Data = [-5.12 5.12];

H = zeros(50,3);
J ={};

for i = 1:50
    [x,fval,exitflag,output,population,score] = GA_solver_code(nvars,lb,ub,PopulationSize_Data,CrossoverFraction_Data,MaxGenerations_Data,MaxStallGenerations_Data,InitialPopulationMatrix_Data);
    % x = optimal solution
    % fval = objection function value
    H(i,1:2)= x;
    H(i,3)  = fval;
    J{i} = output.message;
end
for i = 1:50
    disp(H(i,1:2))
    disp(J{i})
end
