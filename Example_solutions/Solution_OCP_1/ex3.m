% Car A-->B single shooting

clear all
close all
clc

N = 100;  % control discretization
m = 500;  % vehicle mass [kg]

% Create optimization environment
opti = casadi.Opti();

% declare variables
% state
x = opti.variable(1,1); % position
y = opti.variable(1,1);
dx = opti.variable(1,1); % velocity
dy = opti.variable(1,1);
% controls
ddx = opti.variable(N,1); % acceleration
ddy = opti.variable(N,1);
T = opti.variable(1); % motion time

% ODE right hand side
ode = @(x,u)[x(3); x(4); u(1); u(2)];  % =xdot

% input constraints
F_min = -2500.;
F_max = 2500.;

% path constraints
v_min = -10.;
v_max = 10.;

% initial and terminal constraints
x_init = [0., 0., 0., 0.];
x_final = [10., 10., 0., 0.];

x1 = [x(1); y(1); dx(1); dy(1)];
state = [x1];
for k=1:N   
   % shooting constraint
   state_f = rk4(ode,T/N,state(:,end),[ddx(k), ddy(k)]);
   state = [state,state_f];
end

% path constraint
opti.subject_to(v_min<= state(3,:) <= v_max);
opti.subject_to(v_min<= state(4,:) <= v_max);
opti.subject_to(F_min <= m*ddx <= F_max);
opti.subject_to(F_min <= m*ddy <= F_max);
opti.subject_to(T >= 0);

opti.subject_to({state(1,1)==x_init(1), state(1,end)==x_final(1), state(2,1)==x_init(2), state(2,end)==x_final(2)});
opti.subject_to({state(3,1)==x_init(3), state(3,end)==x_final(3), state(4,1)==x_init(4), state(4,end)==x_final(4)});

% set initial guess
opti.set_initial(T, 5); % seconds

% Objective function
opti.minimize(T);

opti.solver('ipopt');
% solve optimization problem
sol = opti.solve();

% retrieve the solution, by rebuilding state vector
state_opt = [sol.value(x); sol.value(x); sol.value(dx); sol.value(dy)];
accx_opt = sol.value(ddx);
accy_opt = sol.value(ddy);
T_opt = sol.value(T);
for k=1:N   
   % shooting constraint
   xf = rk4(ode,T_opt/N,state_opt(:,end),[accx_opt(k), accy_opt(k)]);
   state_opt = [state_opt,xf];
end

% time grid for printing
tgrid = linspace(0,T_opt, N+1);

figure;
title('State trajectories')
subplot(3,1,1)
hold on
plot(tgrid, state_opt(1,:), 'b-x')
plot(tgrid, state_opt(2,:), 'r-o')
xlabel('time [s]')
ylabel('position [m]')

subplot(3,1,2)
hold on
plot(tgrid, state_opt(3,:), 'b-x')
plot(tgrid, state_opt(4,:), 'r-o')
xlabel('time [s]')
ylabel('velocity [m/s]')

subplot(3,1,3)
hold on
stairs(tgrid(1:end-1), m*accx_opt, 'b-x')
stairs(tgrid(1:end-1), m*accy_opt, 'r-o')
xlabel('time [s]')
ylabel('acceleration [m/s^2]')

figure
hold on
plot(state_opt(1,:),state_opt(2,:))
xlabel('position-x [m]')
ylabel('position-y [m]')
title('Top view')
axis equal

disp(strcat('Optimal motion time: ' , num2str(sol.value(T)), ' s'));
