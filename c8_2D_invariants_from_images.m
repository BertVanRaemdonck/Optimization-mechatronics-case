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

opti.subject_to(U(1,:)>=0);

% FS_frame - constrain to be orthogonal (only needed for one timestep, property is propagated by integrator)
opti.subject_to(Rt{1}'*Rt{1} == eye(2));

% Dynamic constraints
for k=1:N
    % Integrate current state to obtain next state
    Xk_end = rk4(ode_simp,dt,X{k},U(:,k));
    
    % Gap closing constraint
    opti.subject_to(Xk_end==X{k+1});
    
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
%%

sol.value(objective_fit)

figure
hold on
plot(meas_pos(1,:),meas_pos(2,:),'b-')

traj = sol.value([p{:}]);
plot(traj(1,:),traj(2,:),'ro')
axis equal

figure
plot(sol.value(U)')

U_sol = sol.value(U);
