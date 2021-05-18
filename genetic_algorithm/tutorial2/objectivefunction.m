function Y = objectivefunction(X)
% Note: X is a [1x2] vector, i.e. X = [x1 x2]

x1 = X(1,1); % x1 = first variable
x2 = X(1,2); % x2 = second variable

A = 10; % parameter of the optimization problem
n = 2;  % parameter of the optimization problem

Y = A*n + [(x1)^2 - A*cos(2*pi*x1)] + [(x2)^2-A*cos(2*pi*x2)];

% Y = Rastrigin function


end