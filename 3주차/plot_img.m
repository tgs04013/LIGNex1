[A,map,none] = imread('missile.png');

%theta = linspace(0,4*pi,200);
image('CData',A,'XData',[0 100],'YData',[0 100])

% hold on
% plot(theta,sin(theta)./theta,'LineWidth',3)
% hold off