
function  [h1 h2 h3 h4] = plotCoM(comx,zmp)

height = 0.5;
h1 = line([0 comx],[0  height],'Color', 'k','LineWidth',10);
hold on
h2 = plot(0,0,'or','LineWidth',10);
h3 = plot(comx,height,'or','LineWidth',10);

h4 = plot(zmp,0,'ob','LineWidth',10);
assignin('base', 'h1', h1)
assignin('base', 'h2', h2) 
assignin('base', 'h3', h3) 
%axis equal
% 
% view(3)
 %view([-90,120,40]) % swap the x and y axis
% 
xlim([-1 ,1])
ylim([0 ,1])

xlabel('X')
ylabel('Y')
end

