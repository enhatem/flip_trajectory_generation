%% cumtrapz results

% tc = step:step:t1;
% tf = (t1+step):step:(t1+t2);
% tr = (t1+t2+step):step:(t1+t2+t3);
% 
% y1 = y(1,1:length(z1));
% y2 = y(1,length(z1)+1:length(z1)+length(z2));
% y3 = y(1,length(z1)+length(z2)+1:length(z1)+length(z2)+length(z3));
% 
% r = 0.02;
% l = 0.05;
% 
% disp('The error on the criterion is: ');
% err = norm(ydd.*cos(phi)+(zdd+gravity).*sin(phi));
% disp(err);
% 
% figure,  plot(tc,y1,'LineWidth',1.5,'Color','b'),
% hold on, plot(tf,y2,'LineWidth',1.5,'Color','r'), 
% hold on, plot(tr,y3,'LineWidth',1.5,'Color','y'),
% hold on, plot(0,0,'*g'), hold on, plot(t1,y1(length(y1)),'*g'),
% hold on, plot(t1+t2,y2(length(y2)),'*g'),
% hold on, plot(t1+t2+t3,y3(length(y3)),'*g'),
% title('Position y(t)'), grid,

y1 = y;

figure
T_cumtrapz = Step:Step:t1+t2+t3;
plot(T_cumtrapz, y1)
title('Postion y(t) with cumtrapz'), grid

%ode45 results
%% integrate ydd to obtain yd

total_time = t1+t2+t3;
n = length(zdd);

gt = linspace(0,total_time,n);
g = -tan(phi).*(zdd + gravity);


% tspan = [0 1.99]; % time span
ic = 0; % initial condition
opts = odeset('RelTol',1e-2,'AbsTol',1e-4); % ode options
[T_yd2,yd2] = ode45(@(T_yd2,yd2) myode_ydd(T_yd2,gt,g), T_cumtrapz, ic, opts);


% plot(t,yd)

%% integrate yd to obtain y

ht = gt;
h = yd2;

% tspan = [0 1.99]; % time span
ic = 0; % initial condition
opts = odeset('RelTol',1e-2,'AbsTol',1e-4); % ode options
[T_y2,y2] = ode45(@(T_y2,y2) myode_yd(T_y2,ht,h), T_cumtrapz, ic, opts);

figure
plot(T_y2,y2)
title('Postion y(t) with ode45'), grid

%% error plot

error = y2.' - y1;

plot(T_y2,error)