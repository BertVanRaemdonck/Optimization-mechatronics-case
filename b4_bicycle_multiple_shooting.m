% Car A-->B multiple shooting

clear all
close all
clc

N = 100; % Control discretization
L = 1;   % Length of the wheel base

% Create optimization environment
opti = casadi.Opti();

% declare variables
% states
x = opti.variable(N+1,1); % position
y = opti.variable(N+1,1);
theta = opti.variable(N+1,1);
% controls
vel = opti.variable(N,1); 	% total vehicle velocity
delta = opti.variable(N,1); % steering angle

T = opti.variable(1); % motion time

% ODE rhs function
% "x" = x, y, theta
% "u" = vel, delta
ode = @(s,u)[u(1)*cos(s(3)) ; u(1)*sin(s(3)) ; u(1)/L*tan(u(2))];  % =xdot

% input constraints
vel_min = 0; % I can't ride backwards on my bike!
vel_max = 5;
% you also need to limit the steering angle, otherwise the steering angle
% can become +/- pi/2 which results in an undefined rotational speed (don't
% try this with you own bike!)
delta_min = -pi/4;
delta_max = pi/4;

% Initial and terminal constraints
x_init = [0., 0., pi/2]; % x, y, theta   
x_final = [10., 10., pi];

% Construct all constraints

for k=1:N
   xk      = [x(k); y(k); theta(k)];
   xk_plus = [x(k+1); y(k+1); theta(k+1)];
   
   % shooting constraint
   xf = rk4(ode,T/N,xk,[vel(k); delta(k)]);
   opti.subject_to(xk_plus==xf);
end

% path constraint
opti.subject_to(vel_min <= vel <= vel_max);
opti.subject_to(delta_min <= delta <= delta_max);
opti.subject_to(T >= 0);

opti.subject_to({x(1)==x_init(1), x(end)==x_final(1), y(1)==x_init(2), y(end)==x_final(2)});
opti.subject_to({theta(1)==x_init(3), theta(end)==x_final(3)});

% set initial guess
opti.set_initial(T, 5); % seconds

%theta_est = atan2(x_final(2)-x_init(2), x_final(1)-x_init(1));
%opti.set_initial(theta, theta_est*ones(size(theta)));
%opti.set_initial(x, linspace(x_init(1), x_final(1), N+1));
%opti.set_initial(y, linspace(x_init(2), x_final(2), N+1));
%opti.set_initial(delta, zeros(size(delta)));
%opti.set_initial(vel, vel_max*ones(size(vel)));

% Objective function
opti.minimize(T);

opti.solver('ipopt')
% solve optimization problem
sol = opti.solve();

% retrieve the solution
posx_opt = sol.value(x);
posy_opt = sol.value(y);
theta_opt = sol.value(theta);
vel_opt = sol.value(vel);
delta_opt = sol.value(delta);
T_opt = sol.value(T);

% time grid for printing
tgrid = linspace(0,T_opt, N+1);

figure;
title('State trajectories')
subplot(2,1,1)
hold on
plot(tgrid, posx_opt, 'b-x')
plot(tgrid, posy_opt, 'r-o')
plot(tgrid, theta_opt, 'g')
xlabel('time [s]')
ylabel('position [m]')

subplot(2,1,2)
hold on
stairs(tgrid(1:end-1), vel_opt, 'b-x')
stairs(tgrid(1:end-1), delta_opt, 'r-o')
xlabel('time [s]')
ylabel('inputs')

figure
hold on
plot(posx_opt,posy_opt)
xlabel('position-x [m]')
ylabel('position-y [m]')
title('Top view')
axis equal

disp(strcat('Optimal motion time: ' , num2str(sol.value(T)), ' s'));
