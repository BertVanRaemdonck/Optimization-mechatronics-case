%% fitting a circle to a set of points

clear all;
close all;
clc;

%% Huber norm

opti = casadi.Opti();

r = 1;

% huber norm parameters
lambda = 1;
mu = 10;

data = [1 0; 2-sqrt(2)/2 sqrt(2)/2; 2 1;2.5 0; 2 -1]';

p = opti.variable(2,1);

% Plot cost function for error vector (x,y)
% note that this is NOT the objective function but just an illustration of
% the shape of the Huber norm of a single vector!

[X,Y] = meshgrid(linspace(-.07,.07), linspace(-.07,.07));
Z = zeros(size(X));

for i = 1:size(X,1)
    for j = 1:size(X,2)
        z = [X(i,j);Y(i,j)];
        one_norm = sum(abs(z),1);
        if one_norm >= lambda/(2*mu)
           Z(i,j) = lambda*(one_norm-lambda/(4*mu)); 
        else
           Z(i,j) = mu*(z'*z); 
        end
    end
end

figure('Name', 'Huber norm of a vector')
subplot(2,1,1)
mesh(X,Y,Z)
rotate3d on
xlabel('x')
ylabel('y')
zlabel('Value of the Huber norm')
subplot(2,1,2)
contour(X,Y,Z,15)
xlabel('x')
ylabel('y')

% create a huber norm lookup table
N = 150; % amount of interpolation points in a direction

data_xmin = min(data(1,:));
data_xmax = max(data(1,:));
data_ymin = min(data(2,:));
data_ymax = max(data(2,:));

xgrid = linspace(data_xmin-1.5*r, data_xmax+1.5*r, N);
ygrid = linspace(data_ymin-1.5*r, data_ymax+1.5*r, N);
[X_lut, Y_lut] = ndgrid(xgrid, ygrid); % interpolation points

huber_lut = zeros(size(X_lut));
for i=1:size(huber_lut,1)
    for j=1:size(huber_lut,2)
        for k = 1:size(data,2)
            difference = data(:,k) - [X_lut(i,j); Y_lut(i,j)];  % vector pointing from the current center point to the current data point
            error = difference - r*difference/norm(difference); % vector pointing from the closest point on the current circle to the current data point
            one_norm = abs(error(1))+abs(error(2));
            if one_norm >= lambda/(2*mu)
                huber_lut(i,j) = huber_lut(i,j) + lambda*(one_norm-lambda/(4*mu));
            else
                huber_lut(i,j) = huber_lut(i,j) + mu*error'*error;
            end
        end
    end
end

LUT = casadi.interpolant('LUT', 'bspline', {xgrid, ygrid}, huber_lut(:));
opti.minimize(LUT([p(1) p(2)]));

% make sure the point stays within the LUT bounds
opti.subject_to(xgrid(1) < p(1) < xgrid(end));
opti.subject_to(ygrid(1) < p(2) < ygrid(end));

% set initial values
opti.set_initial(p{1},(data_xmin+data_xmax)/2);
opti.set_initial(p{2},(data_ymin+data_ymax)/2);

opti.solver('ipopt');

sol = opti.solve();

% plot Huber norm fitted circle
figure('Name', 'Circle fitted to the data using the Huber norm')
hold on
plot(data(1,:),data(2,:),'o')
arc = 0:0.01:2*pi;
circle = plot(sol.value(p(1))+r*cos(arc), sol.value(p(2))+r*sin(arc),'r');
axis equal
legend('data','Huber')

fprintf('Optimal location: (%f,%f)\n', sol.value(p(1)), sol.value(p(2)))

% plot mesh with objective function with Huber norm
figure('Name', 'Cost function with Huber norm')
mesh(X_lut, Y_lut, huber_lut)
xlabel('x')
ylabel('y')
zlabel('Objective with Huber')
axis tight


%% 2 norm for comparison
opti2 = casadi.Opti();       % initializing optimazation problem

x = opti2.variable(1,1);     % initiializing unknown variables
y = opti2.variable(1,1);

opti2.set_initial(x,(data_xmin+data_xmax)/2);
opti2.set_initial(y,(data_ymin+data_ymax)/2);

% data = [1   2-(sqrt(2)/2)   2   3   2;
%         0   sqrt(2)/2       1   0   -1];
data = [1   2-(sqrt(2)/2)   2   2.5   2;
        0   sqrt(2)/2       1   0   -1];

% gradually building the cost function (2 norm) 
objective = 0;

for index=1:1:5
    objective = objective + (sqrt( (x - data(1,index))^2 + (y - data(2,index))^2) - 1)^2;
end

% searching solution
opti2.minimize (objective);
opti2.solver ('ipopt');
sol2 = opti2.solve();
px = sol2.value (x)
py = sol2.value (y)

% plot squared 2 norm fitted circle
r = 1;

figure('Name', 'Circle fitted to data using the squared 2 norm')
plot(data(1,:),data(2,:),'o')

hold on
th = linspace(0,2*pi,100);         
plot(r*cos(th) + px, r*sin(th) + py, 'b')
hold off
axis equal

% plot mesh with objective function with squared 2 norm
N = size(data,2);

Z_2_norm = zeros(size(X_lut));
for i=1:size(X_lut,1)
   for j=1:size(X_lut,2)
      pnum = [X_lut(i,j);Y_lut(i,j)];
      e = sqrt(sum((data-repmat(pnum,1,N)).^2,1))'-r;
      Z_2_norm(i,j) = norm(e,2); 
   end
end

figure
mesh(X_lut,Y_lut,Z_2_norm)
axis tight


%% Comparison of results
figure()
hold on
plot(data(1,:),data(2,:),'o')
arc = 0:0.01:2*pi;
circle = plot(sol.value(p(1))+r*cos(arc), sol.value(p(2))+r*sin(arc),'r');
axis equal
plot(r*cos(th) + px, r*sin(th) + py,'b');
legend('data','Huber','2 norm');

figure
mesh(X_lut,Y_lut,huber_lut)
rotate3d on
hold on
xlabel('x')
ylabel('y')
mesh(X_lut,Y_lut,Z_2_norm)
axis tight

legend('Huber','2 norm');
