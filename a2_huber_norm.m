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


% second attempt at plot

figure()
plot(data(1,:),data(2,:))

hold on
th = linspace(0,2*pi,100);          % begin, einde, aantal punten
plot(r*cos(th) + px, r*sin(th) + py)
hold off



%% Huber norm for a)

% import casadi.*

opti = casadi.Opti();       % initializing optimazation problem

x = opti.variable(1,1);     % initiializing unknown variables
y = opti.variable(1,1);

% data = [1   2-(sqrt(2)/2)   2   3   2;            % normal data
%         0   sqrt(2)/2       1   0   -1];
data = [1   2-(sqrt(2)/2)   2   2.5   2;            % pertubed data
        0   sqrt(2)/2       1   0   -1];


% gradually building the cost function (2 norm) 
var = 0;
mu = 2;
lambda = 1;

for index=1:1:5
    z_vector = [(x - data(1,index))  (y - data(2,index))];
    norm_1 = sum(abs(z_vector));
    norm_2 = sqrt(sum(z_vector.^2));
    
    threshold = lambda/(2*mu);
    logical = (sign(norm_1 - threshold) + 1)/2;
    
    logical_inv = (sign(threshold - norm_1) + 1)/2;
    
    % for 1 norm
    var = var + logical * lambda*(norm_1 - lambda/(4*mu)); 
    % for 2 norm   
    var = var + logical_inv * mu*(norm_2);
        
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


% second attempt at plot

figure()
plot(data(1,:),data(2,:))

hold on
th = linspace(0,2*pi,100);          % begin, einde, aantal punten
plot(r*cos(th) + px, r*sin(th) + py)
hold off


