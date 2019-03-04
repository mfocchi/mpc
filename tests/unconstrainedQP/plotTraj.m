close all
clear all
format = '%f  %f'
[time,zmpRefx] = textread('zmpRef.txt', format);
[time,zmp] =  textread('zmp.txt', format);
[time,jerk] = textread('jerk.txt', format);
[time,com] = textread('com.txt', format);

addpath('../../../../../../install/share/crawl_planner/tests/unconstrainedQP')
addpath('../')
run('../loadFigOptions.m')

%NB trajectory starts from the second value
ha(1)=axes('position',[xgraph three_y1 three_w three_h]);
plot(ha(1),time  ,jerk ,'-k' )
ylabel('jerk  [$m/s^3$]','interpreter','latex','FontSize',20)
grid on
set(ha(1),'XtickLabel',[ ])

ha(2)=axes('position',[xgraph three_y2 three_w three_h]);
plot(ha(2),time  ,com ,'-b' )
ylabel('com [$m$]','interpreter','latex','FontSize',20)
grid on
set(ha(2),'XtickLabel',[ ])

ha(3)=axes('position',[xgraph three_y3 three_w three_h]);
plot(ha(3), time  ,zmpRefx ,'-r' )
hold on
plot(ha(3),time , zmp,'-b' )


ylabel('zmp [$m$]','interpreter','latex','FontSize',20)
legend({'ref','actual'},'interpreter','latex','orientation','vertical','location' ,'southeast','FontSize',20)
xlabel('Time [$s$]','interpreter','latex')

set(gcf, 'Paperunits' , 'centimeters', 'PaperSize', [18 13], 'PaperPosition', [0 0 18  13]);
print(gcf, '-dpdf','../docs/unconstrained_R=1e-06.pdf')
%print(gcf, '-dpdf','../docs/unconstrained_R=1.pdf')

