clear all
clc
close all

% import casadi.*

opti = casadi.Opti();       % initializing optimazation problem

x = opti.variable(1,1);     % initiializing unknown variables
y = opti.variable(1,1);

% data = [1   2-(sqrt(2)/2)   2   3   2;
%         0   sqrt(2)/2       1   0   -1];
data = [1   2-(sqrt(2)/2)   2   2.5   2;
        0   sqrt(2)/2       1   0   -1];

% gradually building the cost function (2 norm) 
var = 0;


for index=1:1:5
    var = var + (sqrt( (x - data(1,index))^2 + (y - data(2,index))^2) - 1)^2;
end

% searching solution
opti.minimize (var);
%opti.subject_to (x^2+y ^2 <=1)
%opti.subject_to (x+y >=0)
opti.solver ('ipopt');
sol = opti.solve();
px = sol.value (x)
py = sol.value (y)


% plotting solution
r = 1;



figure()
plot(data(1,:),data(2,:),'o')

hold on
th = linspace(0,2*pi,100);          % begin, einde, aantal punten
plot(r*cos(th) + px, r*sin(th) + py)
hold off
axis equal

% plotting mesh with 2 norm
[X,Y] = meshgrid(linspace(0,3),linspace(-2,2));
N = size(data,2);           % get the size of the data


Z_2_norm = zeros(size(X));
for i=1:size(X,1)
   for j=1:size(X,2)
      pnum = [X(i,j);Y(i,j)];
      e = sqrt(sum((data-repmat(pnum,1,N)).^2,1))'-r;
      Z_2_norm(i,j) = norm(e,2); 
   end
end

figure
mesh(X,Y,Z_2_norm)




%% Huber norm for a)

% initialize variables
r = 1;          % radius of circle
mu = 10;         % weight of 2 norm
lambda = 1;     % weight of 1 norm



% import casadi.*

opti = casadi.Opti();       % initializing optimazation problem

% data = [1   2-(sqrt(2)/2)   2   3   2;            % normal data
%         0   sqrt(2)/2       1   0   -1];
data = [1   2-(sqrt(2)/2)   2   2.5   2;            % pertubed data
        0   sqrt(2)/2       1   0   -1];

N = size(data,2);           % get the size of the data


p = opti.variable(2,1);     % contains the x,y center we want to calculate
v = opti.variable(N,1);
w = opti.variable(N,1);

s = opti.variable(N,1);     % the slack variable we want to use, cause 1 norm cannot be used in casadi

e = sqrt(sum((data-repmat(p,1,N)).^2,1))'-r;        % the distance we want to minimize

%e1 = lambda*e;                                      % the weighted 1 norm
%e2 = mu*(e'*e);                                     % the weighted 2 norm

%opti.subject_to(-s<=e1<=s);                         % formation of the 1 norm using slack variable

%opti.minimize(sum(s)+e2);                           % minimize the slack variable (1 norm) + 2 norm

norm1 = sum(s);
norm2 = sum(w.^2);

opti.minimize( lambda*norm1 + mu*norm2 );
opti.subject_to( v + w == e);
opti.subject_to( s >= v );
opti.subject_to( s >= -v );

opti.set_initial(p, [1.5;0.5]);                     % set the initial guess

opti.solver('ipopt');

sol = opti.solve();

figure()                            % plot the solution
hold on
plot(data(1,:),data(2,:),'o')
arc = 0:0.01:2*pi;
circle = plot(sol.value(p(1))+r*cos(arc), sol.value(p(2))+r*sin(arc),'r');
axis equal


[X,Y] = meshgrid(linspace(0,3),linspace(-2,2));


Z_H_norm= zeros(size(X));
for i=1:size(X,1)
   for j=1:size(X,2)
      pnum = [X(i,j);Y(i,j)];
      e = sqrt(sum((data-repmat(pnum,1,N)).^2,1))'-r;
      Z_H_norm(i,j) = lambda*norm(e,1)+mu*norm(e,2); 
   end
end

figure
mesh(X,Y,Z_H_norm)


%% Comparison 2 norm with Huber norm

figure()                            % plot the solution
hold on
plot(data(1,:),data(2,:),'o')       % data points

th = linspace(0,2*pi,100);          % begin, einde, aantal punten
plot(r*cos(th) + px, r*sin(th) + py,'g')                % 2 norm

arc = 0:0.01:2*pi;
circle = plot(sol.value(p(1))+r*cos(arc), sol.value(p(2))+r*sin(arc),'r');  % Huber norm

hold off
legend('data','2 norm', 'Huber norm');
axis equal


figure()
mesh(X,Y,Z_2_norm)
hold on
mesh(X,Y,Z_H_norm)
hold off
legend('mesh 2 norm','mesh H norm')



%% old code of Huber norm

% % gradually building the cost function (2 norm) 
% var = 0;
% mu = 2;
% lambda = 1;
% 
% for index=1:1:5
%     z_vector = [(x - data(1,index))  (y - data(2,index))];
%     norm_1 = sum(abs(z_vector));
%     norm_2 = sqrt(sum(z_vector.^2));
%     
%     threshold = lambda/(2*mu);
%     logical = (sign(norm_1 - threshold) + 1)/2;
%     
%     logical_inv = (sign(threshold - norm_1) + 1)/2;
%     
%     % for 1 norm
%     var = var + logical * lambda*(norm_1 - lambda/(4*mu)); 
%     % for 2 norm   
%     var = var + logical_inv * mu*(norm_2);
%         
% end
% 
% % searching solution
% opti.minimize (var);
% %opti.subject_to (x^2+y ^2 <=1)
% %opti.subject_to (x+y >=0)
% opti.solver ('ipopt');
% sol = opti.solve();
% px = sol.value (x)
% py = sol.value (y)
% 
% 
% % plotting solution
% r = 1;
% 
% 
% % second attempt at plot
% 
% figure()
% plot(data(1,:),data(2,:))
% 
% hold on
% th = linspace(0,2*pi,100);          % begin, einde, aantal punten
% plot(r*cos(th) + px, r*sin(th) + py)
% hold off
% 
% 

%% copied code from ex_fit2.m
% 
% % double pendulum with a fixed base
% 
% clear all;
% close all;
% clc;
% 
% opti = casadi.Opti();
% 
% r = 1;
% 
% data = [1 0; 2-sqrt(2)/2 sqrt(2)/2; 2 1; 3 0; 2 -1]';
% data = [1 0; 2-sqrt(2)/2 sqrt(2)/2; 2 1; 2.5 0; 2 -1]';
% N = size(data,2);
% 
% p = opti.variable(2,1);
% 
% s = opti.variable(N,1);
% 
% e = sqrt(sum((data-repmat(p,1,N)).^2,1))'-r;
% 
% opti.subject_to(-s<=e<=s);
% 
% opti.minimize(sum(s));
% 
% opti.set_initial(p, [1.5;0.5]);
% 
% opti.solver('ipopt');
% 
% sol = opti.solve();
% 
% figure()
% hold on
% plot(data(1,:),data(2,:),'o')
% arc = 0:0.01:2*pi;
% circle = plot(sol.value(p(1))+r*cos(arc), sol.value(p(2))+r*sin(arc),'r');
% axis equal
% 
% % % %
% 
% [X,Y] = meshgrid(linspace(0,3),linspace(-2,2));
% 
% 
% Z = zeros(size(X));
% for i=1:size(X,1)
%    for j=1:size(X,2)
%       pnum = [X(i,j);Y(i,j)];
%       e = sqrt(sum((data-repmat(pnum,1,N)).^2,1))'-r;
%       Z(i,j) = norm(e,1); 
%    end
% end
% 
% figure
% mesh(X,Y,Z)
% view([38,10])
% 
% xlabel('x')
% ylabel('y')