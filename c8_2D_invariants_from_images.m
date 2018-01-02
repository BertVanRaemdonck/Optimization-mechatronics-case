clear all;
close all;
clc;

import casadi.*

%% Read trajectory from image sequence
I = imread('Images/Clean/Image0001.png');

red_UINT8 = I(:,:,1);
data = cast(red_UINT8, 'double');
[n_row, n_col] = size(data);

fig = figure();

filename = 'animation_c8.gif';          % added to create a gif

x0 = n_col/2;
y0 = n_row/2;

loops = 100;
F(loops) = struct('cdata',[],'colormap',[]);

list_x = [];
list_y = [];

for i = 1:loops
    I = imread(strcat('Images/Clean/Image',sprintf('%04d',i),'.png'));
    image(I);

    red_UINT8 = I(:,:,1);
    data = cast(red_UINT8, 'double');

    [x_, y_] = tracking_method_3(data, x0, y0)

    list_x = [list_x, x_];
    list_y = [list_y, y_];

    hold on
    plot(list_x, list_y, '+b');
    plot(list_x, list_y, 'b');
    hold off
    
    % put figure in the .gif file
    
    frame = getframe(fig);
    image_gif = frame2im(frame);
    [imind, cm] = rgb2ind(image_gif,256);
    
    if i==1;
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
        
    end
    
    % draw the figure now
    
    drawnow

    x0 = x_;
    y0 = y_;
end

%% Getting invariants of trajectory

T = 1;   % End time
N = length(list_x)-1; % Number of control intervals
dt = T/N;
t = linspace(0,T,N+1); % time vector

meas_pos = [list_x ; list_y];

% System states
p  = SX.sym('p',2,1); % object position
Rt = SX.sym('Rt',2,2); % translational Frenet-Serret frame
x = [p; Rt(:)];

% System controls (invariants)
i1 = SX.sym('i1'); % object translation speed
i2 = SX.sym('i2'); % curvature speed translational Frenet-Serret
u = [i1 ; i2];
nu = size(u,1);

% State dynamics equations of the form: dx/dt = f(x,u,t)
dRt = Rt*[0 -i2 ; i2 0];    % the big difference
dp = Rt*[i1;0];

rhs = [dp; dRt(:)];

% Define ordinary differential equations
ode_simp = Function('ode_simp',{x,u},{rhs});

opti = casadi.Opti();

% Create decision variables and parameters for multipleshooting
p = cell(1,N+1);
Rt = cell(1,N+1);
X = cell(1,N+1);

for k=1:N+1
    % System states
    p{k} = opti.variable(2,1); % object position
    Rt{k}  = opti.variable(2,2); % translational Frenet-Serret frame
    
    X{k} =  [p{k};vec(Rt{k})];
end

U = opti.variable(nu,N);

% A good initialization of the invariant speeds is needed to converge to the proper solution
U_est = [];
for k=1:N
    dist = (meas_pos(1,k+1)-meas_pos(1,k))^2 + (meas_pos(2,k+1)-meas_pos(2,k))^2;
    dist = sqrt(dist);  % distance between two consecutive sample points
    speed_est = dist/dt;
    U_est = [U_est speed_est];
end
opti.set_initial(U(1,:),U_est);

opti.subject_to(U(1,:)>=0);

% FS_frame - constrain to be orthogonal (only needed for one timestep, property is propagated by integrator)
opti.subject_to(Rt{1}'*Rt{1} == eye(2));

% Dynamic constraints
for k=1:N
    % Integrate current state to obtain next state
    Xk_end = rk4(ode_simp,dt,X{k},U(:,k));
    
    % Gap closing constraint
    opti.subject_to(Xk_end-X{k+1}==0);    
end

% Construct objective
objective_fit = 0;
for k=1:N+1
    e = p{k} - meas_pos(:,k); % position error
    objective_fit = objective_fit + e'*e;
end

objective_reg = 0;
for k=1:N-1
    e = U(:,k+1) - U(:,k);
    objective_reg = objective_reg + 1e-4*e'*e;
end

% Initialize states
for k=1:N+1
    opti.set_initial(Rt{k}, eye(2));
    opti.set_initial(p{k}, meas_pos(:,k));
end

opti.minimize(objective_fit + objective_reg);
opti.solver('ipopt');

% Solve the NLP
sol = opti.solve();

% Plot the solution
sol.value(objective_fit)

figure
hold on
mix_factor = linspace(0,1,size(meas_pos,2));
color = [1-mix_factor; zeros(size(mix_factor)) ; mix_factor];
for k = 1:size(meas_pos,2)-1
    line = [meas_pos(:,k) meas_pos(:,k+1)];
    plot(line(1,:),line(2,:),'Color', color(:,k))
end

traj = sol.value([p{:}]);
mix_factor = linspace(0,1,size(traj,2));
color = [1-mix_factor; zeros(size(mix_factor)) ; mix_factor];
for k = 1:size(traj,2)
   scatter(traj(1,k), traj(2,k), 30, color(:,k)','filled'); 
end
%plot(traj(1,:),traj(2,:),'ro')
axis equal

figure
plot(sol.value(U)')

U_sol = sol.value(U);

%% Generate a new trajectory based on the same invariants
U_ref = U_sol;
X_ref = sol.value([X{:}]);

% input parameters
p_fix = [ 0 0 ; -10 10 ; 60 60]';    % positions the trajectory has to visit
t_fix = [ 1   ; N+1    ; round(N/2)]';    % respective "times" at which the trajectory has to visit those positions

opti.subject_to();  % clear the previous constraints so that they don't interfere with the current problem

% repeat the constraints that remain valid
opti.subject_to(U(1,:)>=0);
opti.subject_to(Rt{1}'*Rt{1} == eye(2));
for k=1:N
    % Integrate current state to obtain next state
    Xk_end = rk4(ode_simp,dt,X{k},U(:,k));    
    % Gap closing constraint
    opti.subject_to(Xk_end-X{k+1}==0);    
end
for l = 1:length(t_fix)
    k = t_fix(l);
    opti.subject_to(p{k} - p_fix(:,l) == 0);
end

% Construct objective
objective = 0;
for k=1:N
    e = U(:,k) - U_ref(:,k); % position error
    objective = objective + e'*e;
end

opti.minimize(objective);

% Initialize variables
opti.set_initial(U, U_ref);

% solve the NLP
sol = opti.solve();

% Plot the solution
sol.value(objective_fit)
traj = sol.value([p{:}]);

figure
% the dots are the points of the trajectory: the color is more red at the
% beginning and more blue at the end of the trajectory. The black circles
% represent the points from p_fix.
hold on
mix_factor = linspace(0,1,size(traj,2));
color = [1-mix_factor; zeros(size(mix_factor)) ; mix_factor];
for k = 1:size(traj,2)
   scatter(traj(1,k), traj(2,k), 20, color(:,k)','filled'); 
end

scatter(p_fix(1,:), p_fix(2,:), 60, 'k')
hold off

axis equal