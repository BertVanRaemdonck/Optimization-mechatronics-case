clear;
close all;
clc;

%%

I = imread('Images/Clean/Image0001.png');

f = figure();
image(I);  % put image in figure

red_UINT8 = I(:,:,1);   % extract red channel data from RGB
red = cast(red_UINT8, 'double');  % from uint8 to double


[x,y] = tracking_method_1(red);  % most red pixel
hold on
plot(x,y,'w+')

% [x,y] = tracking_method_2(red);  % column and row with most red pixels
% plot(x,y,'b+')

