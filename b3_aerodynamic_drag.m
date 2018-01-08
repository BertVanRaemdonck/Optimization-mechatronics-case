% Car A-->B multiple shooting

clear all
close all
clc

%% Car A-->B multiple shooting without aerodynamic drag (for reference)

N = 100; % Control discretization
m = 500; % vehicle mass [kg]

% Create optimization environment
opti = casadi.Opti();

% declare variables
% states
x = opti.variable(N+1,1); % position
y = opti.variable(N+1,1);
dx = opti.variable(N+1,1); % velocity
dy = opti.variable(N+1,1);
% controls
ddx = opti.variable(N,1); % acceleration
ddy = opti.variable(N,1);

T = opti.variable(1); % motion time --> will be optimized

% ODE rhs function
ode = @(x,u)[x(3); x(4); u(1) ; u(2)];  % =xdot

% input constraints
F_min = -2500.;
F_max = 2500.;

% Path constraints
v_min = -10.;
v_max = 10.;

% Initial and terminal constraints
x_init = [0., 0., 0., 0.];
x_final = [10., 10., 0., 0.];

% Construct all constraints

for k=1:N
   xk      = [x(k); y(k); dx(k); dy(k)];
   xk_plus = [x(k+1); y(k+1); dx(k+1); dy(k+1)];
   
   % shooting constraint
   xf = rk4(ode,T/N,xk,[ddx(k), ddy(k)]);
   opti.subject_to(xk_plus==xf);
end

% path constraint
opti.subject_to(v_min <= dx <= v_max);
opti.subject_to(v_min <= dy <= v_max);
opti.subject_to(F_min <= m*ddx <= F_max);
opti.subject_to(F_min <= m*ddy <= F_max);
opti.subject_to(T >= 0);

opti.subject_to({x(1)==x_init(1), x(end)==x_final(1), y(1)==x_init(2), y(end)==x_final(2)});
opti.subject_to({dx(1)==x_init(3), dx(end)==x_final(3), dy(1)==x_init(4), dy(end)==x_final(4)});

% set initial guess
opti.set_initial(T, 5); % seconds

% Objective function
opti.minimize(T);

opti.solver('ipopt')
% solve optimization problem
sol = opti.solve();

% retrieve the solution
posx_opt = sol.value(x);
posy_opt = sol.value(y);
velx_opt = sol.value(dx);
vely_opt = sol.value(dy);
accx_opt = sol.value(ddx);
accy_opt = sol.value(ddy);
T_opt = sol.value(T);


%% Plot the reference solution (without drag)

% time grid for printing
tgrid = linspace(0,T_opt, N+1);

% plot the found solution
figure;
subplot(3,1,1)
% plot the found positions
hold on
plot(tgrid, posx_opt, 'b-x')
plot(tgrid, posy_opt, 'r-o')
xlabel('time [s]')
ylabel('position [m]')
title('State trajectories without aerodynamic drag')

subplot(3,1,2)
% plot the found velocities
hold on
plot(tgrid, velx_opt, 'b-x')
plot(tgrid, vely_opt, 'r-o')
xlabel('time [s]')
ylabel('velocity [m/s]')

subplot(3,1,3)
% plot the found accelerations
hold on
stairs(tgrid(1:end-1), m*accx_opt, 'b-x')
stairs(tgrid(1:end-1), m*accy_opt, 'r-o')
xlabel('time [s]')
ylabel('acceleration [m/s^2]')


% plot the top down view of the trajectory
figure
hold on
plot(posx_opt,posy_opt)
xlabel('position-x [m]')
ylabel('position-y [m]')
title('Top down view without aerodynamic drag')
axis equal

% display the found optimal time
disp(strcat('Optimal motion time: ' , num2str(sol.value(T)), ' s'));



%% Car A-->B multiple shooting with aerodynamic drag

clear all

N = 100; % Control discretization
m = 500; % vehicle mass [kg]

% Create optimization environment
opti = casadi.Opti();

% declare variables
% states
x = opti.variable(N+1,1); % position
y = opti.variable(N+1,1);
dx = opti.variable(N+1,1); % velocity
dy = opti.variable(N+1,1);
% controls
ddx = opti.variable(N,1); % acceleration
ddy = opti.variable(N,1);

T = opti.variable(1); % motion time --> will be minimized

% ODE rhs function
ode = @(x,u)[x(3); x(4); u(1) ; u(2)];  % =xdot

% input constraints
F_min = -2500.;
F_max = 2500.;

% Path constraints
v_min = -10.;
v_max = 10.;

% Initial and terminal constraints
x_init = [0., 0., 0., 0.];
x_final = [10., 10., 0., 0.];

% Construct all constraints

for k=1:N
   xk      = [x(k); y(k); dx(k); dy(k)];
   xk_plus = [x(k+1); y(k+1); dx(k+1); dy(k+1)];
   
   % shooting constraint
   xf = rk4(ode,T/N,xk,[ddx(k), ddy(k)]);
   opti.subject_to(xk_plus==xf);
end

% path constraint
c = 50;                  % aerodynamic drag coefficient


opti.subject_to(v_min <= dx <= v_max);
opti.subject_to(v_min <= dy <= v_max);
opti.subject_to(F_min <= m*ddx + c*(dx(1:end-1).^2) <= F_max);         % added aerodynamic drag
opti.subject_to(F_min <= m*ddy + c*(dy(1:end-1).^2) <= F_max);         % added aerodynamic drag
opti.subject_to(T >= 0);

opti.subject_to({x(1)==x_init(1), x(end)==x_final(1), y(1)==x_init(2), y(end)==x_final(2)});
opti.subject_to({dx(1)==x_init(3), dx(end)==x_final(3), dy(1)==x_init(4), dy(end)==x_final(4)});

% set initial guess
opti.set_initial(T, 5); % seconds

% Objective function
opti.minimize(T);

opti.solver('ipopt')
% solve optimization problem
sol = opti.solve();

% retrieve the solution
posx_opt = sol.value(x);
posy_opt = sol.value(y);
velx_opt = sol.value(dx);
vely_opt = sol.value(dy);
accx_opt = sol.value(ddx);
accy_opt = sol.value(ddy);
T_opt = sol.value(T);


%% Plot the solution with drag

% time grid for printing
tgrid = linspace(0,T_opt, N+1);

figure;

subplot(3,1,1)
% plot the found positions
hold on
plot(tgrid, posx_opt, 'b-x')
plot(tgrid, posy_opt, 'r-o')
xlabel('time [s]')
ylabel('position [m]')
title('State trajectories WITH aerodynamic drag')

subplot(3,1,2)
% plot the found velocities
hold on
plot(tgrid, velx_opt, 'b-x')
plot(tgrid, vely_opt, 'r-o')
xlabel('time [s]')
ylabel('velocity [m/s]')

subplot(3,1,3)
% plot the found accelerations
hold on
stairs(tgrid(1:end-1), m*accx_opt, 'b-x')
stairs(tgrid(1:end-1), m*accy_opt, 'r-o')
xlabel('time [s]')
ylabel('acceleration [m/s^2]')


% plot the top down view of the trajectory
figure
hold on
plot(posx_opt,posy_opt)
xlabel('position-x [m]')
ylabel('position-y [m]')
title('Top down view WITH aerodynamic drag')
axis equal

% display the found optimal time
disp(strcat('Optimal motion time: ' , num2str(sol.value(T)), ' s'));
