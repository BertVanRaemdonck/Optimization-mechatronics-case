clear all
clc
close all

% This file does the fitting of the circle to a set of data points using
% the Huber norm. The Huber norm is implemented with the method described
% in:
% Zadorozhnyi, O. e.a., 'Huber-Norm Regularization for Linear Prediction
% Models', http://www.stephanmandt.com/papers/ECML_2016.pdf
% In particular, the method described in section 3.2 of that paper is
% implemented here

%% Squared 2 norm for reference

opti = casadi.Opti();       % initializing optimization problem

x = opti.variable(1,1);     % initiializing unknown variables
y = opti.variable(1,1);

% data = [1   2-(sqrt(2)/2)   2   3   2;
%         0   sqrt(2)/2       1   0   -1];      % unperturbed circle points data
data = [1   2-(sqrt(2)/2)   2   2.5   2;
        0   sqrt(2)/2       1   0   -1];        % perturbed circle points data

% gradually building the cost function (2 norm) 
objective = 0;
for index=1:1:5
    objective = objective + (sqrt( (x - data(1,index))^2 + (y - data(2,index))^2) - 1)^2;
end

% searching solution
opti.minimize (objective);
opti.solver ('ipopt');
sol = opti.solve();
px = sol.value (x)
py = sol.value (y)

% plotting solution
r = 1;

figure('Name', 'Fitted circle using the squared 2 norm')
plot(data(1,:),data(2,:),'o')

hold on
th = linspace(0,2*pi,100);         
plot(r*cos(th) + px, r*sin(th) + py)
hold off
axis equal

% plotting mesh with 2 norm
[X,Y] = meshgrid(linspace(0,3),linspace(-2,2));
N = size(data,2);          

Z_2_norm = zeros(size(X));
for i=1:size(X,1)
   for j=1:size(X,2)
      pnum = [X(i,j);Y(i,j)];
      e = sqrt(sum((data-repmat(pnum,1,N)).^2,1))'-r;
      Z_2_norm(i,j) = norm(e,2); 
   end
end

figure('Name', 'Objective function with the squared 2 norm')
mesh(X,Y,Z_2_norm)

%% Huber norm implemented as suggested by the paper

% initialize variables
r = 1;          % radius of circle
mu = 10;         % weight of 2 norm
lambda = 1;     % weight of 1 norm

opti = casadi.Opti();       % initializing optimazation problem

% data = [1   2-(sqrt(2)/2)   2   3   2;            % normal data
%         0   sqrt(2)/2       1   0   -1];
data = [1   2-(sqrt(2)/2)   2   2.5   2;            % pertubed data
        0   sqrt(2)/2       1   0   -1];

N = size(data,2);

p = opti.variable(2,1);     % contains the x,y center we want to calculate
v = opti.variable(N,1);     % part of the error of which we will take the 1 norm
w = opti.variable(N,1);     % part of the error of which we will take the squared 2 norm

s = opti.variable(N,1);     % slack variable to determine the absolute values needed for the 1 norm

e = sqrt(sum((data-repmat(p,1,N)).^2,1))'-r;         % the distance we want to minimize

norm1 = sum(s);
norm2 = sum(w.^2);

opti.minimize( lambda*norm1 + mu*norm2 );
opti.subject_to( v + w == e);
opti.subject_to( s >= v );
opti.subject_to( s >= -v );

opti.set_initial(p, [1.5;0.5]);                     % set the initial guess

opti.solver('ipopt');

sol = opti.solve();

figure('Name', 'Fitted circle using the Huber norm')  % plot the solution
hold on
plot(data(1,:),data(2,:),'o')
arc = 0:0.01:2*pi;
circle = plot(sol.value(p(1))+r*cos(arc), sol.value(p(2))+r*sin(arc),'r');
axis equal

% plotting mesh with Huber norm
[X,Y] = meshgrid(linspace(0,3),linspace(-2,2));

Z_H_norm = zeros(size(X));
for i=1:size(Z_H_norm,1)
    for j=1:size(Z_H_norm,2)
        for k = 1:size(data,2)
            error = sqrt(sum((data(:,k)-[X(i,j); Y(i,j)]).^2,1))'-r;
            one_norm = abs(error);
            if one_norm >= lambda/(2*mu)
                Z_H_norm(i,j) = Z_H_norm(i,j) + lambda*(one_norm-lambda/(4*mu));
            else
                Z_H_norm(i,j) = Z_H_norm(i,j) + mu*error^2;
            end
        end
    end
end

figure('Name', 'Objective function with the Huber norm')
mesh(X,Y,Z_H_norm)


%% Comparison 2 norm with Huber norm

figure('Name', 'Comparison solution Huber and squared 2 norm')
hold on
plot(data(1,:),data(2,:),'o')       % data points

th = linspace(0,2*pi,100);          
plot(r*cos(th) + px, r*sin(th) + py,'g') % 2 norm

arc = 0:0.01:2*pi;
circle = plot(sol.value(p(1))+r*cos(arc), sol.value(p(2))+r*sin(arc),'r');  % Huber norm

hold off
legend('data','2 norm', 'Huber norm');
axis equal

figure('Name', 'Comparison cost functions Huber and squared 2 norm')
mesh(X,Y,Z_2_norm)
hold on
mesh(X,Y,Z_H_norm)
hold off
legend('mesh 2 norm','mesh H norm')

