close all
clear all
format = '%f  %f'

addpath('~')
addpath('../')

[time,zmpRefx] = textread('zmpRef_x.txt', format);
[time,zmpRefy] = textread('zmpRef_y.txt', format);
[time,jerk_x] = textread('jerk_x.txt', format);
[time,jerk_y] = textread('jerk_y.txt', format);
[time,zmp_x] =  textread('zmp_x.txt', format);
[time,zmp_y] =  textread('zmp_y.txt', format);

[time,zmp_x] =  textread('zmp_x.txt', format);
[time,zmp_y] =  textread('zmp_y.txt', format);

[time,com_x] =  textread('com_x.txt', format);
[time,com_y] =  textread('com_y.txt', format);

close all

run('../loadFigOptions.m')

%NB trajectory starts from the second value
ha(1)=axes('position',[xgraph three_y1 three_w three_h]);
plot(time  ,jerk_x ,'-k' );grid on;hold on
plot(time  ,jerk_y ,'-r' );grid on;
ylabel('jerk  [$m/s^3$]','interpreter','latex','FontSize',20)
legend({'X','Y'},'interpreter','latex','orientation','vertical','location' ,'southeast','FontSize',20)
grid on
set(ha(1),'XtickLabel',[ ])

ha(2)=axes('position',[xgraph three_y2 three_w three_h]);
plot(ha(2),com_x  ,com_y ,'-b' )
ylabel('com [$m$]','interpreter','latex','FontSize',20)
grid on
set(ha(2),'XtickLabel',[ ])

ha(3)=axes('position',[xgraph three_y3 three_w three_h]);
plot(ha(3), zmpRefx  ,zmpRefy ,'-r' )
hold on
plot(ha(3),zmp_x , zmp_y,'-b' )

xlim([zmp_x(1) zmp_x(end)])

ylabel('zmp [$m$]','interpreter','latex','FontSize',20)
legend({'ref','actual'},'interpreter','latex','orientation','vertical','location' ,'southeast','FontSize',20)
xlabel('Time [$s$]','interpreter','latex')

set(gcf, 'Paperunits' , 'centimeters', 'PaperSize', [18 13], 'PaperPosition', [0 0 18  13]);
print(gcf, '-dpdf','../docs/unconstrainedXY.pdf')


