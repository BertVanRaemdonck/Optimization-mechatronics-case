clear all;
close all;
clc;

import casadi.*

T = 1;   % End time
N = 100; % Number of control intervals
dt = T/N;
t = linspace(0,T,N+1); % time vector

fit = load('fit');
U_ref = fit.U_sol;

meas_pos = [t;0.1*t.*sin(4*pi*t);0.1*t.*cos(4*pi*t)];

%% Building obstacle
% building cube
obs1_anker = [0.5,0.75,0.5];
obs1_side = 0.25;
number_obs_points = 8;

obs1_points = zeros(number_obs_points,3);
% building all points
for index = 1:number_obs_points
    % build ground plane first and increase x first before y
    if mod(index,2) == 0        % even numbers
        delta_x = obs1_side;
    else 
        delta_x = 0;
    end
    
    if index == 3 | index == 4 | index == 7 | index == 8
        delta_y = obs1_side;
    else
        delta_y = 0;
    end
    
    if index >= 5
        delta_z = obs1_side;
    else
        delta_z = 0;
    end
    obs1_points(index,:) = obs1_anker + [delta_x, delta_y, delta_z];
end

% plotting the cube for testing

        % plotcube(obs1_anker,[obs1_side,obs1_side,obs1_side] ,.8,[1 0 0]);       % anker, size sides, transparancy, colour in rgb
% enlarging obs1_points for plotting
obs1_points(9:12,:) = obs1_points(1:4,:);
        
figure();
for index = 1: number_obs_points
    a=1;
    index_numbers = [index,index+a];
    plot3(obs1_points(index_numbers,1),obs1_points(index_numbers,2),obs1_points(index_numbers,3),'b');
    hold on
    
    b = 2;
    index_numbers = [index,index+b];
    plot3(obs1_points(index_numbers,1),obs1_points(index_numbers,2),obs1_points(index_numbers,3),'b');

    c = 4;
    index_numbers = [index,index+c];
    plot3(obs1_points(index_numbers,1),obs1_points(index_numbers,2),obs1_points(index_numbers,3),'b');
end
hold off
xlabel('x')
ylabel('y')
zlabel('z')


% building sphere
obs_sphere_center = [0.5;0.75;0.5];
obs_sphere_radius = 0.4;
%% Building casadi problem


% System states
p = SX.sym('p',3,1); % object position
Rt  = SX.sym('Rt' ,3,3); % translational Frenet-Serret frame
x = [p;Rt(:)];

% System controls (invariants)
i1 = SX.sym('i1'); % object translation speed
i2 = SX.sym('i2'); % curvature speed translational Frenet-Serret
i3 = SX.sym('i3'); % torsion speed translational Frenet-Serret
u = [i1 ; i2 ; i3];
nu = size(u,1);

% State dynamics equations of the form: dx/dt = f(x,u,t)
dRt = Rt*skew([i3;i2;0]);
dp = Rt*[i1;0;0];

rhs = [dp;dRt(:)];

% Define ordinary differential equations
ode_simp = Function('ode_simp',{x,u},{rhs});

opti = casadi.Opti();

% Create decision variables and parameters for multipleshooting
p = cell(1,N+1);
Rt = cell(1,N+1);
X = cell(1,N+1);

for k=1:N+1
    % System states
    p{k} = opti.variable(3,1); % object position
    Rt{k}  = opti.variable(3,3); % translational Frenet-Serret frame
    
    X{k} =  [p{k};vec(Rt{k})];
end

U = opti.variable(nu,N);

opti.subject_to(U(1,:)>=0); % Can only move forward

% FS_frame - constrain to be orthogonal (only needed for one timestep, property is propagated by integrator)
opti.subject_to(Rt{1}'*Rt{1} == eye(3));

P_start = [0;1;0];
P_end = [1;1;1];

opti.subject_to(p{1}==P_start);
opti.subject_to(p{end}==P_end);

% Dynamic constraints
for k=1:N
    % Integrate current state to obtain next state
    Xk_end = rk4(ode_simp,dt,X{k},U(:,k));
    
    % Gap closing constraint
    opti.subject_to(Xk_end==X{k+1});
    
    % obstacle avoidance constraint
    opti.subject_to(norm(p{k}-obs_sphere_center,2) > obs_sphere_radius);
    % opti.subject_to((x(index)-obs_pos_moving(index,1)).^2 + (y(index)-obs_pos_moving(index,2)).^2 >= obs_rad_moving^2);
    
end

% Construct objective
objective = 0;
for k=1:N
    e = U(:,k) - U_ref(:,k); % position error
    objective = objective + e'*e;
end

opti.set_initial(U, U_ref);

% Initialize states
for k=1:N+1
    opti.set_initial(Rt{k}, eye(3));
end

opti.minimize(objective);
opti.solver('ipopt');

% Solve the NLP
sol = opti.solve();
%%

figure
hold on
plot3(meas_pos(1,:),meas_pos(2,:),meas_pos(3,:),'b-')

traj = sol.value([p{:}]);
plot3(traj(1,:),traj(2,:),traj(3,:),'ro')
plot3(P_start(1),P_start(2),P_start(3),'kx')
plot3(P_end(1),P_end(2),P_end(3),'ks')
axis equal

% for index = 1: number_obs_points
%     a=1;
%     index_numbers = [index,index+a];
%     plot3(obs1_points(index_numbers,1),obs1_points(index_numbers,2),obs1_points(index_numbers,3),'b');
%     hold on
%     
%     b = 2;
%     index_numbers = [index,index+b];
%     plot3(obs1_points(index_numbers,1),obs1_points(index_numbers,2),obs1_points(index_numbers,3),'b');
% 
%     c = 4;
%     index_numbers = [index,index+c];
%     plot3(obs1_points(index_numbers,1),obs1_points(index_numbers,2),obs1_points(index_numbers,3),'b');
% end

[X_sphere,Y_sphere,Z_sphere] = sphere;
surf(obs_sphere_radius*X_sphere + obs_sphere_center(1,1), ...
    obs_sphere_radius*Y_sphere + obs_sphere_center(2,1),...
    obs_sphere_radius*Z_sphere + obs_sphere_center(3,1));
xlabel('x')
ylabel('y')
zlabel('z')


view([-76 14])
figure
hold on
plot(sol.value(U)')
plot(U_ref')
