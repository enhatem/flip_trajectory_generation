clear; close all; clc;

%% Drone parameters and costants

global g t_step z1_min z2_min z3_min z1_max z2_max z3_max m Ixx l u1_max u2_max

% Drone parameters
m =  29e-3 / 2; % mass
Ixx =  1.657171e-05; % Inertia
l = 0.046; % arm length

% Constants
t_step = 0.01; % 100 Hz
g = 9.81; % m/s^2

% Bounds on the z trajectory of the reaching phase
z1_min = 0.8;
z1_max = 2;

% Bounds on the z trajectory of the flipping phase
z2_min = 0.8;
z2_max = 2;

% Bounds on the z trajectory of the recovery phase
z3_min = 0.8;
z3_max = 2;

% Maximum thrust and torque reachable by the drone
u1_max = 0.9 * ( ( 46e-3 * g ) / 2 ); % Maximum thrust 
u2_max = 0.1 * ( 1 / 2 * u1_max * l); % Maximum torque

%% Constraints applied on the optimization problem

% lower bound on the initial phi angle at the start of the reaching phase
phi_start_min = 0;

% upper bound on the final phi angle at the end of the reaching phase
phi_start_max = pi/2;

% lower bound on the initial phi angle at the end of the flipping phase (start of the recovery phase)
phi_end_min = 3/2*pi;

% upper bound on the final phi angle at the end of the recovery phase
phi_end_max   = 2*pi;

% bounds on t1 (time of the reaching phase trajectory)
t1_min = 0.1;
t1_max = inf;

% bounds on t2 (time of the flipping phase trajectory)
t2_min = 0.1;
t2_max = inf;

% bounds on t3 (time of the recovery phase trajectory)
t3_min = 0.1;
t3_max = inf;

% lower and upper bounds
lb = [ z1_min   z1_min   z2_min  z3_min  phi_start_min  phi_end_min  t1_min  t2_min  t3_min ];
ub = [ z1_max   z1_max   z2_max  z3_max  phi_start_max  phi_end_max  t1_max  t2_max  t3_max ];

% nonlinear bounds
nl_con = @nonlinear_bounds;

% objective function
obj = @objective_function;

%% Optimization problem

% % Initial condition
% x0 = [ 1.1    1.49    1.1    0.8    pi/2-0.2    (3/2+0.2)*pi    0.4   0.3  0.4 ];
% options  = optimset('Display', 'iter', 'Tolx', 1e-14, 'Tolfun',...
%                     1e-14, 'MaxIter', 1e20, 'MaxFunEvals', 1e20);
% 
% solution = fmincon(obj,x0,[],[],[],[],lb,ub,nl_con,options);
% 
% z1 = solution(1);
% z2 = solution(2);
% z3 = solution(3);
% z4 = solution(4);
% phi_start = solution(5);
% phi_end   = solution(6);
% t1 = round(solution(7),2);
% t2 = round(solution(8),2);
% t3 = round(solution(9),2);
%% Solving the optimization problem using genetic algorithm

% seed used for reproducibility
rng default

% Genetic algorithm parameters
nvars = 9;
PopulationSize_Data = 900;
CrossoverFraction_Data = 0.7;
MaxStallGenerations_Data = 100;

% number of times the ga solver will run
iter =100;

% Variables to store the solutions and output message
H = zeros(iter,10);
J = {};

for i=1:iter
    [solution,fval,exitflag,output] = ga_solver_code(nvars,lb,ub,PopulationSize_Data,CrossoverFraction_Data,MaxStallGenerations_Data);
    H(i,1:9)=solution;             % storing the solution
    H(i,10)=fval;           % storing the function value
    J{i} = output.message;   % storing the output message
    
    fprintf('The number of generations is: %d\n', output.generations);

    fprintf('The number of function evaluations is: %d\n', output.funccount);

    fprintf('The best function value found is: %g\n', fval);
    
    z1          = solution(1);
    z2          = solution(2);
    z3          = solution(3);
    z4          = solution(4);
    phi_start   = solution(5);
    phi_end     = solution(6);
    t1          = round(solution(7),2);
    t2          = round(solution(8),2);
    t3          = round(solution(9),2);

    build_trajectory;
    %visualize_trajectory;
end


% z1 = solution(1);
% z2 = solution(2);
% z3 = solution(3);
% z4 = solution(4);
% phi_start = solution(5);
% phi_end   = solution(6);
% t1 = round(solution(7),2);
% t2 = round(solution(8),2);
% t3 = round(solution(9),2);



%% Building trajectory

build_trajectory;
plot_data;