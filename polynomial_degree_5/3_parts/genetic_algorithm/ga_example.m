xi = linspace(-6,2,300);
yi = linspace(-4,4,300);
[X,Y] = meshgrid(xi,yi);
Z = ps_example([X(:),Y(:)]);
Z = reshape(Z,size(X));
surf(X,Y,Z,'MeshStyle','none')
colormap 'jet'
view(-26,43)
xlabel('x(1)')
ylabel('x(2)')
title('ps\_example(x)')

%% Find minimum of this function using ga

A = [-1 -1];
b = -1;
Aeq = [-1 1];
beq = 5;

options = optimoptions('ga','ConstraintTolerance',1e-6,'PlotFcn', @gaplotbestf);


rng default % For reproducibility
fun = @ps_example;
x = ga(fun,2,A,b,Aeq,beq,[],[],[],options)
