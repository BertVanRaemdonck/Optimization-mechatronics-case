clear all;
close all;
clc;

%% Setting up the possible cases

% change the following cases to true if you want to test them and plot them

case_small = false;
case_big = true;

case_sphere = true;
case_tetrahedron = false;       % the big tetrahedron does not work for the moment, if a tolerance is added to the inequality constraint, it might work better
case_cube = false;              % does not work for the moment BUT is useful since the vertices of the tetrahedron is a subset of those of the cube

%% Setting up the experiment
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
if case_big
    obs1_anker = [0.5,0.75,0.5];
    obs1_side = 0.4;
end

if case_small
    obs1_anker = [0.5,0.75,0.5];
    obs1_side = 0.25;
end
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

if case_cube        
    % enlarging obs1_points for plotting
    obs1_points(9:16,:) = obs1_points(1:8,:);

    figure('Name', 'Obstacle cube');
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

        d = 5;
        index_numbers = [index,index+d];
        plot3(obs1_points(index_numbers,1),obs1_points(index_numbers,2),obs1_points(index_numbers,3),'b');
    end
    hold off
    xlabel('x')
    ylabel('y')
    zlabel('z')
end

% building tetrahedron

if case_tetrahedron
    obs_tetrahedron = [obs1_points(1:3,:);
                        obs1_points(5,:)];

    volume_tetrahedron = calculate_connecting_volume_tetrahedron(obs_tetrahedron);

    % plotting tetrahedron
    figure('Name', 'Obstacle tetrahedron')
    plot3(obs_tetrahedron(:,1),obs_tetrahedron(:,2),obs_tetrahedron(:,3),'b');
    hold on
    plot3(obs_tetrahedron([1,3],1),obs_tetrahedron([1,3],2),obs_tetrahedron([1,3],3),'b');
    plot3(obs_tetrahedron([1,4],1),obs_tetrahedron([1,4],2),obs_tetrahedron([1,4],3),'b');
    plot3(obs_tetrahedron([2,4],1),obs_tetrahedron([2,4],2),obs_tetrahedron([2,4],3),'b');
    xlabel('x')
    ylabel('y')
    zlabel('z')
end

if case_sphere
    % building sphere
    obs_sphere_center = [0.5;0.75;0.5];
    obs_sphere_radius = 0.4;
end

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

% Create decision variables and parameters for multiple shooting
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

% Dynamic and obstacle avoidance constraints
for k=1:N
    % Integrate current state to obtain next state
    Xk_end = rk4(ode_simp,dt,X{k},U(:,k));
    
    % Gap closing constraint
    opti.subject_to(Xk_end==X{k+1});
    
    % obstacle avoidance constraint
    if case_sphere
        opti.subject_to(norm(p{k}-obs_sphere_center,2) > obs_sphere_radius);
    end
    
    if case_tetrahedron
        volume_subject = calculate_connecting_volume_tetrahedron(obs_tetrahedron,p{k}');
        opti.subject_to(volume_subject - volume_tetrahedron > 0);
    end
    
    % no case for the cube since it doesn't work, but the cube case needs
    % to be turned on to provide the source of the vertices of the 
    % tetrahedron
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

%% Plot the solution

figure('Name', 'Reference trajectory, newly generated trajectory and obstacle')
hold on
% plot the reference trajectory
plot3(meas_pos(1,:),meas_pos(2,:),meas_pos(3,:),'b-')

traj = sol.value([p{:}]);
% plot the newly found trajectory
plot3(traj(1,:),traj(2,:),traj(3,:),'ro')
% plot the start and finish location that the new trajectory had to obey
plot3(P_start(1),P_start(2),P_start(3),'kx')
plot3(P_end(1),P_end(2),P_end(3),'ks')
axis equal

% plotting the cube if needed
if case_cube        
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

        d = 5;
        index_numbers = [index,index+d];
        plot3(obs1_points(index_numbers,1),obs1_points(index_numbers,2),obs1_points(index_numbers,3),'b');
    end
    hold off
    xlabel('x')
    ylabel('y')
    zlabel('z')

end

% plotting the sphere if needed

if case_sphere
    [X_sphere,Y_sphere,Z_sphere] = sphere;
    surf(obs_sphere_radius*X_sphere + obs_sphere_center(1,1), ...
        obs_sphere_radius*Y_sphere + obs_sphere_center(2,1),...
        obs_sphere_radius*Z_sphere + obs_sphere_center(3,1));
end

% plotting the tetrahedron if needed

if case_tetrahedron
    for i = 1:4
        face_coords = obs_tetrahedron;
        face_coords(i,:) = [];  % delete a row to reduce the coordinate matrix to only contain coordinates of a certain face
        fill3(face_coords(:,1), face_coords(:,2), face_coords(:,3), ones(3,1))
    end

    xlabel('x')
    ylabel('y')
    zlabel('z')
end


view([-76 14])

% plotting the invariants
figure('Name', 'Invariants of the newly generated trajectory')
hold on
plot(sol.value(U)')
plot(U_ref')
