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

% moving obstacle parameters
situation = 2;
    if situation == 1
        obs_moving_start = [-1,1];
        obs_moving_end = [10,9];
        obs_rad_moving = 1;
    else
        obs_moving_start = [9,7];
        obs_moving_end = [-1,1];
        obs_rad_moving = 2;
    end

% constructing the (linear) trajectory of the obstacle
obs_pos_moving = [linspace(obs_moving_start(1), obs_moving_end(1), N) ;
                  linspace(obs_moving_start(2), obs_moving_end(2), N)  ]';

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

% for moving object
for index=1:N       % loop over entire time
    opti.subject_to((x(index)-obs_pos_moving(index,1)).^2 + (y(index)-obs_pos_moving(index,2)).^2 >= obs_rad_moving^2);
end

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

fig = figure('Name', 'Trajectory animation');
axis tight manual

filename = 'animation_b1.gif';
%set(gca,'nextplot','replacechildren','visible','off')
% to make sure the car does not clip the circle
radius_bol = 0.0;

for index = 1:N
    plot(posx_opt(index),posy_opt(index),'b*')
    hold on
    plot(posx_opt(1:end),posy_opt(1:end),'b')
    hold on
    arc = 0:0.01:2*pi;
    circle = plot(obs_pos_moving(index,1)+(obs_rad_moving-radius_bol)*cos(arc), obs_pos_moving(index,2)+(obs_rad_moving-radius_bol)*sin(arc),'r');
    xlim([-5,15])
    ylim([-5,15])
    hold off
    
    frame = getframe(fig);
    image = frame2im(frame);
    [imind, cm] = rgb2ind(image,256);
    
    if index ==1;
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
        
    end
end

hold off
axis equal

figure('Name', 'Optimal trajectory');
plot(tgrid, posx_opt, 'b')
hold on
plot(tgrid, posy_opt, 'g')
xlabel('Time')
ylabel('Optimal trajectory (x in blue, y in green)')

figure('Name', 'Optimal speed')
plot(tgrid, velx_opt, 'b')
hold on
plot(tgrid, vely_opt, 'g')
xlabel('Time')
ylabel('Optimal speed (x in blue, y in green)')

figure('Name', 'Optimal acceleration')
plot(tgrid(1:end-1), accx_opt, 'b')
hold on
plot(tgrid(1:end-1), accy_opt, 'g')
xlabel('Time')
ylabel('Optimal acceleration (x in blue, y in green)')
