function draw_center( f, x, y )
figure(f);
hold on
plot([x,x],[y-10,y+10],'w');
plot([x-10,x+10],[y,y],'w');
hold off
end

