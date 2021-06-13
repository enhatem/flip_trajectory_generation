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
z2 = polyval(coeff_z2,Step:Step:t2);
zd2 = polyval(coeff_zd2,Step:Step:t2);
zdd2 = polyval(coeff_zdd2,Step:Step:t2);

coeff_phi2 = [(phi_end-phi_start)/t2 phi_start];
coeff_phid2 = polyder(coeff_phi2);
coeff_phidd2 = polyder(coeff_phid2);
phi2 = polyval(coeff_phi2,Step:Step:t2);
phid2 = polyval(coeff_phid2,Step:Step:t2);
phidd2 = polyval(coeff_phidd2,Step:Step:t2);


%% Recovery phase

z3_start = [z_end ((z_start-z_end)/t2-g*t2/2) -g 0 0];
z3_end = [z_hover2 0 0 0 0];
z3 = trajectory(z3_start,z3_end,t3);

phi3_start = [phi_end (phi_end-phi_start)/t2 0 0 0];
phi3_end = [2*flips*pi 0 0 0 0];
phi3 = trajectory(phi3_start,phi3_end,t3);

%% Global trajectory

z = [z1(1,:) z2 z3(1,:)];
zd = [z1(2,:) zd2 z3(2,:)];
zdd = [z1(3,:) zdd2 z3(3,:)];

phi = [phi1(1,:) phi2 phi3(1,:)];
phid = [phi1(2,:) phid2 phi3(2,:)];
phidd = [phi1(3,:) phidd2 phi3(3,:)];

gravity = g*ones(size(z));

% t_Steps = 0.01:0.01:t1+t2+t3;

n = length(zdd);

t_steps = linspace(0,t1+t2+t3,n);

ydd = -tan(phi).*(zdd + gravity);


% %% manual numerical integration (very bad)
% yd = integrate(ydd,0);
% y = integrate(yd,0);

% %% numerical integration with (cumtrapz)
% yd0 = 0; % initial condition for yd
% y0  = 0; % initial condition for y
% yd = yd0 + cumtrapz(t_Steps,ydd);
% y = y0 + cumtrapz(t_Steps,yd);


%% integrate ydd to obtain yd

total_time = t1+t2+t3;
n = length(zdd);

gt = linspace(0,total_time,n);  % function evaluation times
g = -tan(phi).*(zdd + gravity); % function: ydd = g = -tan(phi).*(zdd + gravity)

yd0 = 0; % initial condition
% opts = odeset('RelTol',1e-2,'AbsTol',1e-4); % ode options
[T_yd,yd] = ode45(@(T_yd,yd) myode_ydd(T_yd,gt,g), t_steps, yd0);

%% integrate yd to obtain y

ht = gt;
h = yd;

y0 = 0; % initial condition
% opts = odeset('RelTol',1e-2,'AbsTol',1e-4); % ode options
[T_y,y] = ode45(@(T_y,y) myode_yd(T_y,ht,h), t_steps, y0);

y = y.';
%% calculate the required thrust u1 and torque u2 throughout the trajectory

u1 = m*(zdd+gravity)./cos(phi);
u2 = Ixx*phidd;

