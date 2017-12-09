function [x,y] = tracking_method_3( data ,x0 ,y0)
% x0 and y0 are the initial quesses
[n_row, n_col] = size(data);
[X,Y] = meshgrid(1:n_col, 1:n_row);  % grid with indices

opti = casadi.Opti();

x = opti.variable();
y = opti.variable();        

f = ((X-x).^2 + (Y-y).^2).*data;  % find optimal position of circle in data
f = sum_square(f);  % Opti functionality

opti.minimize(f);
opti.solver('ipopt');

opti.set_initial(x,x0) 
opti.set_initial(y,y0)

sol = opti.solve();

x = sol.value(x);
y = sol.value(y);
end
