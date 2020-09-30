close all
clear all
format = '%f  %f'

addpath('../../../../../../install/share/crawl_planner/tests/constrainedQP_decoupled')
addpath('../')

[time,min_x] = textread('min_x.txt', format);
[time,min_y] = textread('min_y.txt', format);
[time,max_x] = textread('max_x.txt', format);
[time,max_y] = textread('max_y.txt', format);
[time,zmp_x] =  textread('zmp_x.txt', format);
[time,zmp_y] =  textread('zmp_y.txt', format);
[time,com_x] =  textread('com_x.txt', format);
[time,com_y] =  textread('com_y.txt', format);

close all

run('../loadFigOptions.m')

%NB trajectory starts from the second value
ha(1)=axes('position',[xgraph two_y1 two_w two_h]);

plot(ha(1),time  , min_x,'-r' )
hold on
plot(ha(1),time  , max_x,'-r' )
grid on
plot(ha(1),time  , zmp_x,'-b' )
plot(ha(1),time  , com_x,'-k' )
legend({'min','max','zmp','com'},'interpreter','latex','orientation','vertical','location' ,'southeast','FontSize',20)
ylabel('X  [$m/s^3$]','interpreter','latex','FontSize',20)
set(ha(1),'XtickLabel',[ ])
ylim([0 3])

ha(2)=axes('position',[xgraph two_y2 two_w two_h]);

grid on
plot(ha(2),time  , min_y,'-r' )
hold on
plot(ha(2),time  , max_y,'-r' )
grid on
plot(ha(2),time  , zmp_y,'-b' )
plot(ha(2),time  , com_y,'-k' )
xlabel('Time [$s$]','interpreter','latex')
ylabel('Y [$m$]','interpreter','latex','FontSize',20)
set(gcf, 'Paperunits' , 'centimeters', 'PaperSize', [18 13], 'PaperPosition', [0 0 18  13]);
%print(gcf, '-dpdf','../docs/constrainedBox.pdf')
print(gcf, '-dpdf','../docs/constrainedBoxSlacks.pdf')


% 
% figure
% plot(zmp_x  , zmp_y,'-b' );grid on;


