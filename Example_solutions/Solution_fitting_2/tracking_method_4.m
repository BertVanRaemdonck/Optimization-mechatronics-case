function [x,y,r] = tracking_method_4( data )


f_debug = figure;

imshow(data,[-1 1])

[n_row, n_col] = size(data);

[X,Y] = meshgrid(1:n_col, 1:n_row);

opti = splines.Opti();


x = opti.variable();
y = opti.variable();     
r = opti.variable();    
data = (data/max(max(data)))*2.0 -1 ;

kernel = @(x_,y_,r_) tanh(-(((X-x_).^2 + (Y-y_).^2) / r_ - r_));
f_ = (kernel(x,y,r) - data).^2;
f = sum_square(f_);

% build constraints

opti.minimize(f);
opti.subject_to(x>=0);
opti.subject_to(y>=0);
opti.subject_to(x<= size(data,1));
opti.subject_to(y<= size(data,2));
opti.subject_to(10<=r);

opti.solver('ipopt');


opti.callback(@() draw_circle(f_debug, opti.debug.value(x), opti.debug.value(y), opti.debug.value(r)))

[x0,y0] =  tracking_method_3(data);  % generate initial guess using previous method
x0 = size(data,2)/2;
y0 = size(data,1)/2;

opti.set_initial(x,x0)
opti.set_initial(y,y0)
opti.set_initial(r,100)

sol = opti.solve();

x = sol.value(x);
y = sol.value(y);
r = sol.value(r);
 
fig = figure;
subplot(311)
imshow(data,[-1 1])
colorbar

subplot(312)
imshow(kernel(x,y,r),[-1 1])
colorbar

subplot(313)
imshow(kernel(x,y,r)-data,[-1 1])
colorbar

f = sol.value(f);
end
