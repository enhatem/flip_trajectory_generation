function dydt = myode(t,gt,g)
g = interp1(gt,g,t); % Interpolate the data set (gt,g) at time t
dydt = g; % Evaluate ODE at time t