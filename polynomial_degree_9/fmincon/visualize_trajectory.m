tc = Step:Step:t1;
tf = (t1+Step):Step:(t1+t2);
tr = (t1+t2+Step):Step:(t1+t2+t3);

y1 = y(1,1:length(z1));
y2 = y(1,length(z1)+1:length(z1)+length(z2));
y3 = y(1,length(z1)+length(z2)+1:length(z1)+length(z2)+length(z3));

r = 0.02;
l = 0.05;


% Trajectory plot
figure
plot(y1,z1(1,:),'LineWidth',1.5,'Color','b'),
hold on, plot(y2,z2(1,:),'LineWidth',1.5,'Color','r'),
hold on, plot(y3,z3(1,:),'LineWidth',1.5,'Color','y'),
hold on, plot(0,z_hover1,'*g'), hold on, plot(y1(length(y1)),z_start,'*g'),
hold on, plot(y2(length(y2)),z_end,'*g'),
hold on, plot(y3(length(y3)),z_hover2,'*g'),
for i = 1:10:length(y)
    
   t = phi(i);
   
   q1 = y(i)-l*cos(t);
   q2 = y(i)+l*cos(t);
   w1 = z(i)-l*sin(t);
   w2 = z(i)+l*sin(t);
   
   r1y = q1+r*cos(t+pi/2);
   r1z = w1+r*sin(t+pi/2);
   r2y = q2+r*cos(t+pi/2);
   r2z = w2+r*sin(t+pi/2);

   plot([q1 q2],[w1 w2],'k','LineWidth',1.5), hold on
   plot([q1 r1y],[w1 r1z],'k','LineWidth',1.5), hold on
   plot([q2 r2y],[w2 r2z],'k','LineWidth',1.5), hold on
   
end
title('Planar flipping trajectory'), daspect([1 1 1]), grid;

