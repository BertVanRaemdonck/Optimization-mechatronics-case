clear;
close all;
clc;

I = imread('Images/Clean/Image0001.png');

%%
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
    
%  I = imread(strcat('Images/Noise/Image',sprintf('%04d',i),'.png'));
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
    F(i) = getframe(fig);
x0 = x_;
y0 = y_;

end

fig = figure;
movie(fig,F,2)