function dydt = myode_yd(t,ht,h)
h = interp1(ht,h,t); % Interpolate the data set (ht,h) at time t
dydt = h; % Evaluate ODE at time t