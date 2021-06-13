clear; 
close all;
clc;

%% Constant Parameters

global g t_step z1_min z2_min z3_min z1_max z2_max z3_max y_min y_max m Ixx l flips u1_max u1_min u2_max u2_min

% Drone parameters
m =  29e-3 / 2; % mass
Ixx =  1.657171e-05; % Inertia
l = 0.046;% arm length

% Constants
t_step = 0.01; % 100 Hz
g = 9.81;    % m/s^2

% Bounds on the z trajectory of the reaching phase
z1_min = 0.8;
z1_max = 1.5;

% Bounds on the z trajectory of the flipping phase
z2_min = 0.8;
z2_max = 1.5;

% Bounds on the z trajectory of the recovery phase
z3_min = 0.8;
z3_max = 1.5;

% Bound on y
y_min = 0;
y_max = 2;

% Maximum thrust and torque reachable by the drone
u1_max = 0.9 * ( ( 46e-3 * g ) / 2 ); % Maximum thrust 
u1_min = 0;                           % Minimum thrust
u2_max = 0.1 * ( 1 / 2 * u1_max * l); % Maximum torque
u2_min = -u2_max;                     % Minimum torque

% Number of flips to be perfomed by the drone
flips  = 1;

%% Constraints applied on the optimization problem

% lower bound on the initial phi angle at the start of the reaching phase
phi_reaching_start = 0;

% upper bound on the final phi angle at the end of the reaching phase
phi_reaching_end   = pi/2;

% lower bound on the initial phi angle at the end of the flipping phase (start of the recovery phase)
phi_recovery_start = (2*(flips-1)*pi+3/2*pi);

% upper bound on the final phi angle at the end of the recovery phase
phi_recovery_end   = 2*flips*pi;

% bounds on t1 (time of the reaching phase trajectory)
t1_min = 1e-1;
t1_max = inf;

% bounds on t2 (time of the flipping phase trajectory)
t2_min = 1e-2;
t2_max = inf;

% bounds on t3 (time of the recovery phase trajectory)
t3_min = 1e-1;
t3_max = inf;

% lower and upper bounds
lb = [ z1_min   z1_min   z2_min  z3_min  phi_reaching_start  phi_recovery_start  t1_min  t2_min  t3_min ];
ub = [ z1_max   z1_max   z2_max  z3_max  phi_reaching_end    phi_recovery_end    t1_max  t2_max  t3_max ];

% nonlinear bounds
nonlcon = @NL_bounds;

% objective function
fun = @objective_function;

%% Solving the optimization problem using genetic algorithm

% seed used for reproducibility
rng default

% Genetic algorithm parameters
nvars = 9;
PopulationSize_Data = 900;
CrossoverFraction_Data = 0.7;
MaxStallGenerations_Data = 100;

% number of times the ga solver will run
iter =50;

% Variables to store the solutions and output message
H = zeros(iter,10);
J = {};

for i=1:iter
    [x,fval,exitflag,output] = ga_solver_code(nvars,lb,ub,PopulationSize_Data,CrossoverFraction_Data,MaxStallGenerations_Data);
    H(i,1:9)=x;             % storing the solution
    H(i,10)=fval;           % storing the function value
    J{i} = output.message;   % storing the output message
    
    fprintf('The number of generations is: %d\n', output.generations);

    fprintf('The number of function evaluations is: %d\n', output.funccount);

    fprintf('The best function value found is: %g\n', fval);
    
    z_hover1    = x(1);
    z_start     = x(2);
    z_end       = x(3);
    z_hover2    = x(4);
    phi_start   = x(5);
    phi_end     = x(6);
    t1          = x(7);
    t2          = x(8);
    t3          = x(9);
    
    build_trajectory;
    visualize_trajectory;
end


%% 
x = H(45,1:9)
z_hover1    = x(1);
z_start     = x(2);
z_end       = x(3);
z_hover2    = x(4);
phi_start   = x(5);
phi_end     = x(6);
t1          = x(7);
t2          = x(8);
t3          = x(9);
    
build_trajectory;
visualize_trajectory;
figure, plot(u1), title('u1(t)')
%%

% options = optimoptions('ga','ConstraintTolerance',1e-20, ...
%                        'PlotFcn', @gaplotbestf, ...
%                        'Display','iter');
% [x,fval,exitflag,output,population,scores] = ga(fun,9,[],[],[],[],lb,ub,nonlcon,options);

% z_hover1    = x(1);
% z_start     = x(2);
% z_end       = x(3);
% z_hover2    = x(4);
% phi_start   = x(5);
% phi_end     = x(6);
% t1          = x(7);
% t2          = x(8);
% t3          = x(9);

%% Building and plotting the trajectory
% z_hover1    = x(1);
% z_start     = x(2);
% z_end       = x(3);
% z_hover2    = x(4);
% phi_start   = x(5);
% phi_end     = x(6);
% t1          = x(7);
% t2          = x(8);
% t3          = x(9);
% build_trajectory;
% visualize_results;
% visualize_trajectory;
%%

% %% Write trajectory to file

% py = y';
% pz = z';
% roll = phi';
% vy = yd';
% vz = zd';
% rolld = phid';
% 
% ref_X = [py pz roll vy vz rolld];
% ref_U = [u1' u2'];
% %% 
% dlmwrite('saved_data/measX.csv',ref_X);
% dlmwrite('saved_data/simU.csv',ref_U);
