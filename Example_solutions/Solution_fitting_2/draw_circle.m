function  draw_circle( f,x,y,r,color )
if nargin < 5
   color = 'b'; 
end
figure(f);
hold on
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp,'color', color);
% plot(x+xp*0.99,y+yp*0.99,'w');
hold off
end

