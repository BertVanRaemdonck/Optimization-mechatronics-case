% double pendulum with a fixed base

clear all;
close all;
clc;
opti = casadi.Opti();

r = 1;

data = [1 0; 2-sqrt(2)/2 sqrt(2)/2; 2 1; 3 0; 2 -1]';
data = [1 0; 2-sqrt(2)/2 sqrt(2)/2; 2 1; 2.5 0; 2 -1]';
N = size(data,2);

p = opti.variable(2,1);

e = sqrt(sum((data-repmat(p,1,N)).^2,1))'-r;

opti.minimize(e'*e);

opti.solver('ipopt');

sol = opti.solve();

figure()
hold on
plot(data(1,:),data(2,:),'o')
arc = 0:0.01:2*pi;
circle = plot(sol.value(p(1))+r*cos(arc), sol.value(p(2))+r*sin(arc),'r');
axis equal

fprintf('Optimal location: (%f,%f)\n', sol.value(p(1)), sol.value(p(2)))

%%

[X,Y] = meshgrid(linspace(0,3),linspace(-2,2));

for pow=1:2
    Z = zeros(size(X));
    for i=1:size(X,1)
       for j=1:size(X,2)
          pnum = [X(i,j);Y(i,j)];
          e = sqrt(sum((data-repmat(pnum,1,N)).^2,1))'-r;
          Z(i,j) = norm(e,2)^pow; 
       end
    end

    figure
    mesh(X,Y,Z)
    view([38,10])

    xlabel('x')
    ylabel('y')


    set(gcf,'paperunits','centimeters');
    set(gcf,'paperorientation','landscape');
    set(gcf,'papersize',[29.7 21]/2);
    set(gcf,'paperposition',[1 1 28.7 20]/2);
    print(['../../seminar-text/figures/fit' num2str(pow)],'-dpdf')
end