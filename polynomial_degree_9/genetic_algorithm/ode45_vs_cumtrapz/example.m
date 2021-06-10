
ydd = -tan(phi).*(zdd + gravity);

t = linspace(0,t1+t2+t3,length(ydd));

%% cumtrapz
yd0 = 0; % initial condition for yd
y0  = 0; % initial condition for y
yd = yd0 + cumtrapz(t,ydd);
y = y0 + cumtrapz(t,yd);


figure, plot(t,y,'LineWidth',1.5), title('y(t) trajectory'), hold on
%% ode45

%% integrate ydd to obtain yd

total_time = t1+t2+t3;
n = length(zdd);

t_steps = linspace(0,t1+t2+t3,n);


ht = linspace(0,total_time,n);  % function evaluation times
h = -tan(phi).*(zdd + gravity); % function: ydd = g = -tan(phi).*(zdd + gravity)

yd0 = 0; % initial condition
% opts = odeset('RelTol',1e-2,'AbsTol',1e-4); % ode options
[T_yd,yd] = ode45(@(T_yd,yd) myode_ydd(T_yd,ht,h), t_steps, yd0);

%% integrate yd to obtain y

kt = ht;
k = yd;

% tspan = [0 1.99]; % time span
y0 = 0; % initial condition
% opts = odeset('RelTol',1e-2,'AbsTol',1e-4); % ode options
[T_y,y] = ode45(@(T_y,y) myode_yd(T_y,kt,k), t_steps, y0);

%% 
plot(t_steps,y,'--','LineWidth',2), legend('cumtrapz','ode45')
xlabel('time[s]')
ylabel('y[m]')
hold off