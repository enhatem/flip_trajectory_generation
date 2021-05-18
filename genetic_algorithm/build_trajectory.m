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
z2 = polyval(coeff_z2,step:step:t2);
zd2 = polyval(coeff_zd2,step:step:t2);
zdd2 = polyval(coeff_zdd2,step:step:t2);

coeff_phi2 = [(phi_end-phi_start)/t2 phi_start];
coeff_phid2 = polyder(coeff_phi2);
coeff_phidd2 = polyder(coeff_phid2);
phi2 = polyval(coeff_phi2,step:step:t2);
phid2 = polyval(coeff_phid2,step:step:t2);
phidd2 = polyval(coeff_phidd2,step:step:t2);


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

ydd = -tan(phi).*(zdd + gravity);
yd = integrate(ydd,0);
y = integrate(yd,0);

u1 = m*(zdd+gravity)./cos(phi);
u2 = Ixx*phidd;



M = [1 1; 
    l -l];

F = M \ [u1;u2];

f1 = F(1,:);
f2 = F(2,:);
