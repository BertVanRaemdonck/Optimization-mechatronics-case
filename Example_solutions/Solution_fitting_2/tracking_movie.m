I = imread('Images/Clean/Image0001.png');
%  I = imread('Images/Noise/Image0002.png');

meco_binaries('cpp_splines','develop')

%%

fig = figure();
opti = splines.OptiSpline();

x = opti.var();
y = opti.var();     
r = opti.var(); 

red_UINT8 = I(:,:,1);
data = cast(red_UINT8, 'double');
[n_row, n_col] = size(data);

[X,Y] = meshgrid(1:n_col, 1:n_row);

mask = @(x_,y_,r_) (-(((X-x_).^2 + (Y-y_).^2) / r_ - r_));
smooth_step = @(x_,y_,r_) tanh(mask(x_,y_,r_));


[x0,y0,r0] = track_method_4(data)


[x02,y02,r02] = track_method_4(data.*(mask(x0,y0,r0*1.1)<0))

loops = 100;
F(loops) = struct('cdata',[],'colormap',[]);

list_x = [];
list_y = [];
list_r = [];

list_x2 = [];
list_y2 = [];
list_r2 = [];

for i = 1:loops
    
 I = imread(strcat('Images/Noise/Image',sprintf('%04d',i),'.png'));
 I = imread(strcat('Images/Clean/Image',sprintf('%04d',i),'.png'));
image(I);

red_UINT8 = I(:,:,1);
data = cast(red_UINT8, 'double');

  
data = (data/max(max(data)))*2.0 -1 ;

f_= (smooth_step(x,y,r) - data).^2;
f=f_.sum_square() ;

%build constraints

% con = {x>=0, y>=0 , x<= size(data,1), y <= size(data,2), r2>=10, r2<=10000};
con = {r >= 10};
sol = opti.solver(f, con, 'ipopt');
 
sol.value(x,x0)
sol.value(y,y0)
sol.value(r,r0)

sol.solve();

x_ = sol.value(x);
y_ = sol.value(y);
r_ = sol.value(r);

draw_center(fig,x_,y_)

draw_circle(fig,x_,y_,r_)
drawnow
    F(i) = getframe(fig);
x0 = x_;
y0 = y_;
r0 = r_;


list_x = [list_x, x_];
list_y = [list_y, y_];
list_r = [list_r, r_];


sol.value(x,x02)
sol.value(y,y02)
sol.value(r,r02)

sol.solve();

x_ = sol.value(x);
y_ = sol.value(y);
r_ = sol.value(r);

draw_center(fig,x_,y_)

draw_circle(fig,x_,y_,r_)
drawnow
    F(i) = getframe(fig);
x02 = x_;
y02 = y_;
r02 = r_;


list_x2 = [list_x, x_];
list_y2 = [list_y, y_];
list_r2 = [list_r, r_];
end

fig = figure;
movie(fig,F,2)
t = 1:loops;
plot([(sin(t / 8) * 0.2);(cos(t / 16) * 0.2);(cos(t / 16) * 0.2)]')

figure()
plot([list_x;list_y;1./sqrt(list_r.^2 + 1)]') 

figure()
plot([(cos(t / 16) * 0.2);10./sqrt(list_r.^2 + 1)]') 