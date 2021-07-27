v = VideoWriter('missile.avi');

open(v);
x = 0:pi/100:2*pi;

for t = 1:100
 x_t = x + (0.1*t);
 y = sin(x_t);
 plot(x,y);
 image('CData',A,'XData',[x x+100],'YData',[y y+100])
 F = getframe(gca);
writeVideo(v,F);
end
close(v);