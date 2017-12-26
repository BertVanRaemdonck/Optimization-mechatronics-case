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

% building stationary convex obstacle
number_of_edges = 3;
distance_move = [-3,0];

obs_edge_1_start = [3,3] + distance_move;
obs_edge_1_end = [4,3] + distance_move;

obs_edge_2_start = obs_edge_1_end;
obs_edge_2_end = [5,6] + distance_move;

obs_edge_3_start = obs_edge_2_end;
obs_edge_3_end = [3,6] + distance_move;

obs_edge_4_start = obs_edge_3_end;
obs_edge_4_end = obs_edge_1_start;

obs_edge_matrix = [obs_edge_1_start;
                    obs_edge_1_end;
                    obs_edge_2_start;
                    obs_edge_2_end;
                    obs_edge_3_start;
                    obs_edge_3_end;
                    obs_edge_4_start;
                    obs_edge_4_end];


% obs_pos_moving_speed = (obs_moving_end-obs_moving_start)/N;     % average speed of object
% obs_pos_moving = obs_moving_start;
% for index=2:N
%     next_obs_pos = obs_pos_moving(index-1,:) + obs_pos_moving_speed;         % index approx time moment
%     obs_pos_moving = [obs_pos_moving;    next_obs_pos];
% end




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
%opti.subject_to((x-obs_pos(1)).^2 + (y-obs_pos(2)).^2 >= obs_rad^2);
%opti.subject_to((x-obs_pos1(1)).^2 + (y-obs_pos1(2)).^2 >= obs_rad1^2);
%opti.subject_to((x-obs_pos2(1)).^2 + (y-obs_pos2(2)).^2 >= obs_rad2^2);

opti.subject_to(T >= 0);

% for moving object
% for index=1:N       % loop over entire time
%     opti.subject_to((x(index)-obs_pos_moving(index,1)).^2 + (y(index)-obs_pos_moving(index,2)).^2 >= obs_rad_moving^2);
% end

% for index=1:N       % loop over entire time
%     opti.subject_to((x(index)-obs_pos_moving(index,1)).^2 + (y(index)-obs_pos_moving(index,2)).^2 >= obs_rad_moving^2);
%    
% end

% for stationary, convex object
threshold = 0;


% P1 = (x1,y1) first point edge
% P2 = (x2,y2) second point edge
% P0 = (x0,y0) point of vehicle
for index_edge = 1:number_of_edges      % loop over the edges   
    for index=1:N       % loop over entire time
        distance_num = 0;
        distance_num = distance_num + obs_edge_matrix((2*index_edge),2) - obs_edge_matrix((2*index_edge)-1,2)*x(index);  % (y2 - y1)*x0
        distance_num = distance_num - obs_edge_matrix((2*index_edge),1) - obs_edge_matrix((2*index_edge)-1,1)*y(index);  % (x2 - x1)*y0
        distance_num = distance_num + (obs_edge_matrix((2*index_edge),1)*obs_edge_matrix((2*index_edge)-1,2));    % x2*y1
        distance_num = distance_num - (obs_edge_matrix((2*index_edge),2)*obs_edge_matrix((2*index_edge)-1,1));    % y2*x1
        
        distance_den = 0;
        distance_den = distance_den + (obs_edge_matrix((2*index_edge),2) - obs_edge_matrix((2*index_edge)-1,2))^2;   % (y2-y1)^2
        distance_den = distance_den + (obs_edge_matrix((2*index_edge),1) - obs_edge_matrix((2*index_edge)-1,1))^2;   % (x2-x1)^2
                
        distance = abs(distance_num)/sqrt(distance_den);
        %(x(index)-obs_pos_moving(index,1)).^2 + (y(index)-obs_pos_moving(index,2)).^2
        opti.subject_to( distance >= threshold);
        
    end
end


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

fig = figure;
axis tight manual

filename = 'animation_b2.gif';
%set(gca,'nextplot','replacechildren','visible','off')

%circle = plot(obs_pos(1)+obs_rad*cos(arc), obs_pos(2)+obs_rad*sin(arc),'r');
% circle = plot(obs_pos1(1)+obs_rad1*cos(arc), obs_pos1(2)+obs_rad1*sin(arc),'r');
% hold on
% circle = plot(obs_pos2(1)+obs_rad2*cos(arc), obs_pos2(2)+obs_rad2*sin(arc),'r');

% to make sure the car does not clip the circle
radius_bol = 0.0;

for index = 1:N
    plot(posx_opt(index),posy_opt(index),'b*')
    hold on
    plot(posx_opt(1:end),posy_opt(1:end),'b')
    hold on
    arc = 0:0.01:2*pi;
    %next_obs_pos = obs_moving_start + (index * obs_pos_moving_speed);
    %circle = plot(next_obs_pos(1)+obs_rad_moving*cos(arc), next_obs_pos(2)+obs_rad_moving*sin(arc),'r');
    %circle = plot(obs_pos_moving(index,1)+(obs_rad_moving-radius_bol)*cos(arc), obs_pos_moving(index,2)+(obs_rad_moving-radius_bol)*sin(arc),'r');
    line1 = plot([obs_edge_matrix(1,1) obs_edge_matrix(2,1)],[obs_edge_matrix(1,2) obs_edge_matrix(2,2)]);
    line2 = plot([obs_edge_matrix(3,1) obs_edge_matrix(4,1)],[obs_edge_matrix(3,2) obs_edge_matrix(4,2)]);
    line3 = plot([obs_edge_matrix(5,1) obs_edge_matrix(6,1)],[obs_edge_matrix(5,2) obs_edge_matrix(6,2)]);
    line4 = plot([obs_edge_matrix(7,1) obs_edge_matrix(8,1)],[obs_edge_matrix(7,2) obs_edge_matrix(8,2)]);
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

%axis equal

figure
plot(tgrid, velx_opt, 'b')
hold on
plot(tgrid, vely_opt, 'g')

figure
plot(tgrid(1:end-1), accx_opt, 'b')
hold on
plot(tgrid(1:end-1), accy_opt, 'g')
