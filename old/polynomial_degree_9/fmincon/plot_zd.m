zd = [z1(2,:) zd2 z3(2,:)];


T1 = linspace(0,t1, length(z1(2,:)));
T2 = linspace(t1,t1+t2, length(zd2));
T3 = linspace(t1+t2,t1+t2+t3, length(z3(2,:)));
T  = linspace(0,t1+t2+t3, length(zd));

figure, plot(T1,z1(2,:), 'LineWidth',1.5), hold on
plot(T2,zd2, 'LineWidth',1.5), hold on
plot(T3,z3(2,:), 'LineWidth',1.5), hold off 
title('vz(t)')
xlabel('time[s]')
ylabel('vz[m/s]')

figure, plot(T,zd)