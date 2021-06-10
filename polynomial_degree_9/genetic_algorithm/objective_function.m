function J = objective_function(x)
    
    global g t_step flips m
    
    z_hover1    = x(1);
    z_start     = x(2);
    z_end       = x(3);
    z_hover2    = x(4);
    phi_start   = x(5);
    phi_end     = x(6);
    t1          = x(7);
    t2          = x(8);
    t3          = x(9);



    %% Reaching phase

    z1_start = [z_hover1 0 0 0 0];
    z1_end = [z_start ((z_start-z_end)/t2 + g*t2/2) -g 0 0];
    z1 = trajectory(z1_start,z1_end,t1);

    phi1_start = [0 0 0 0 0];
    phi1_end = [phi_start (phi_end-phi_start)/t2 0 0 0];
    phi1 = trajectory(phi1_start,phi1_end,t1);

    %% Flip phase

    coeff_z2 = [-g/2 ((z_end-z_start)/t2+g*t2/2) z_start];
    coeff_zd2 = polyder(coeff_z2);
    coeff_zdd2 = polyder(coeff_zd2);
    z2 = polyval(coeff_z2,t_step:t_step:t2);
    zdd2 = polyval(coeff_zdd2,t_step:t_step:t2);

    coeff_phi2 = [(phi_end-phi_start)/t2 phi_start];
    phi2 = polyval(coeff_phi2,t_step:t_step:t2);

    %% Recovery phase

    z3_start = [z_end ((z_start-z_end)/t2-g*t2/2) -g 0 0];
    z3_end = [z_hover2 0 0 0 0];
    z3 = trajectory(z3_start,z3_end,t3);

    phi3_start = [phi_end (phi_end-phi_start)/t2 0 0 0];
    phi3_end = [2*flips*pi 0 0 0 0];
    phi3 = trajectory(phi3_start,phi3_end,t3);

    %% Global trajectory

    z = [z1(1,:) z2 z3(1,:)];
    zdd = [z1(3,:) zdd2 z3(3,:)];

    phi = [phi1(1,:) phi2 phi3(1,:)];

    gravity = g*ones(size(zdd));
    
    u1 = m*(zdd+gravity)./cos(phi);
    
    n = length(u1);
    
    t = linspace(0,t1+t2+t3,n);

    % Calculating and integrating ydd twice to find the trajectory along y

    ydd = -tan(phi).*(zdd + gravity);
    yd0 = 0; % initial condition for yd
    y0  = 0; % initial condition for y
    yd = yd0 + cumtrapz(t,ydd);
    y = y0 + cumtrapz(t,yd);

%     %% integrating ydd to obtain yd
% 
%     total_time = t1+t2+t3;
%     n = length(zdd);
% 
%     gt = linspace(0,total_time,n);  % function evaluation times
%     g_t = -tan(phi).*(zdd + gravity); % function: ydd = g = -tan(phi).*(zdd + gravity)
% 
%     yd0 = 0; % initial condition
%     % opts = odeset('RelTol',1e-2,'AbsTol',1e-4); % ode options
%     [T_yd,yd] = ode45(@(T_yd,yd) myode_ydd(T_yd,gt,g_t), t, yd0);
% 
%     %% integrate yd to obtain y
% 
%     ht = gt;
%     h = yd;
% 
%     % tspan = [0 1.99]; % time span
%     y0 = 0; % initial condition
%     % opts = odeset('RelTol',1e-2,'AbsTol',1e-4); % ode options
%     [T_y,y] = ode45(@(T_y,y) myode_yd(T_y,ht,h), t, y0);
% 
%     y = y.';
% 
%     % J = trapz(t,z)^2 + trapz(t,y)^2 + trapz(t,u1)^2;
%     
    J = trapz(t,y)^2 + trapz(t,z)^2 + 50*trapz(t,u1)^2;
    
    % objective function: J = t1 + t2 + t3
    % J = trapz(t,u1); % x(7) + x(8) + x(9);
    % J = x(7) + x(8) + x(9);
end
