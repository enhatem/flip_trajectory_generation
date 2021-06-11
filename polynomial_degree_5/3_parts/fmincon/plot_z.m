z = [z1(1,:) z2 z3(1,:)];


T1 = linspace(0,t1, length(z1(1,:)));
T2 = linspace(t1,t1+t2, length(z2));
T3 = linspace(t1+t2,t1+t2+t3, length(z3(1,:)));
T  = linspace(0,t1+t2+t3, length(z));

figure, plot(T1,z1(1,:), 'LineWidth',1.5), hold on
plot(T2,z2, 'LineWidth',1.5), hold on
plot(T3,z3(1,:), 'LineWidth',1.5), hold off 
title('z(t)')
xlabel('time[s]')
ylabel('z[m]')

figure, plot(T,zd)