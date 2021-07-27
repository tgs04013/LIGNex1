%# control animation speed  
DELAY = 1;  
numPoints = 10000;  

%create data  

f = 5;
C = 1;
%C = 1-exp(-f);  
%y = C*(1-(exp(-f*x)));

%# plot graph  
figure('DoubleBuffer','on')                  %# no flickering  
plot(x,y, 'LineWidth',2), grid on  
xlabel('x'), ylabel('y'), title('')  

%# create moving point + coords text  
hLine = line('XData',x(1), 'YData',y(1), 'Color','r', ...  
       'Marker','o', 'MarkerSize',10, 'LineWidth',2);  
hTxt = text(x(1), y(1), sprintf('(%.3f,%.3f)',x(1),y(1)), ...  
    'Color',[0.2 0.2 0.2], 'FontSize',8, ...  
    'HorizontalAlignment','left', 'VerticalAlignment','top');  

line('XData',x_target, 'YData',y_target, 'Color','b', ...  
       'Marker','o', 'MarkerSize',10, 'LineWidth',2);  

%# infinite loop  
v = VideoWriter('missile.avi');
open(v);
i = 1;                                       %# index  
while true     
    %# update point & text  
    set(hLine, 'XData',x(i), 'YData',y(i))     
    set(hTxt, 'Position',[x(i) y(i)], ...  
        'String',sprintf('(%.f,%.f)',[x(i) y(i)]))          
    drawnow                                  %# force refresh  
    %pause(DELAY)                           %# slow down animation  

    i = rem(i+1,numPoints)+1;                %# circular increment  
    frame = getframe(gcf);
    writeVideo(v,frame);
    if i>size(x), break; end        %# in case you close the figure  

end

close(v);