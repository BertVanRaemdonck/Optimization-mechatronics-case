clear all
close all
clc

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

T = opti.variable(1); % motion time

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

% obstacle info
obs_pos = [5,5];
obs_rad = 1;
obs_pos1 = [3,2];
obs_rad1 = 1;
obs_pos2 = [6,7];
obs_rad2 = 1;


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
opti.subject_to((x-obs_pos(1)).^2 + (y-obs_pos(2)).^2 >= obs_rad^2);
opti.subject_to(T >= 0);

opti.subject_to({x(1)==x_init(1), x(end)==x_final(1), y(1)==x_init(2), y(end)==x_final(2)});
opti.subject_to({dx(1)==x_init(3), dx(end)==x_final(3), dy(1)==x_init(4), dy(end)==x_final(4)});

opti.minimize(T);

% set initial guess
opti.set_initial(x, 0);
opti.set_initial(y, 0);
opti.set_initial(dx, v_max/2);
opti.set_initial(dy, v_max/2);
opti.set_initial(T, 5) % seconds

% set state bounds
opti.subject_to(-10<=x<=20);
opti.subject_to(-10<=y<=20);

% solve optimization problem
opti.solver('ipopt');
sol = opti.solve();

% retrieve the solution
posx_opt = sol.value(x);
posy_opt = sol.value(y);
velx_opt = sol.value(dx);
vely_opt = sol.value(dy);
accx_opt = sol.value(ddx);
accy_opt = sol.value(ddy);
T_opt = sol.value(T);

% time grid for printing
tgrid = linspace(0,T_opt, N+1);

figure;
plot(tgrid, posx_opt, 'b')
hold on
plot(tgrid, posy_opt, 'g')    

figure
plot(posx_opt,posy_opt,'b-o')
hold on
arc = 0:0.01:2*pi;
circle = plot(obs_pos(1)+obs_rad*cos(arc), obs_pos(2)+obs_rad*sin(arc),'r');
% circle = plot(obs_pos1(1)+obs_rad1*cos(arc), obs_pos1(2)+obs_rad1*sin(arc),'r');
% circle = plot(obs_pos2(1)+obs_rad2*cos(arc), obs_pos2(2)+obs_rad2*sin(arc),'r');

axis equal

figure
plot(tgrid, velx_opt, 'b')
hold on
plot(tgrid, vely_opt, 'g')

figure
plot(tgrid(1:end-1), accx_opt, 'b')
hold on
plot(tgrid(1:end-1), accy_opt, 'g')
