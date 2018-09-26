close all
clear all
clc
format short g
format_input = '%f  %f';
format_input2 = '%f  %f %f';

    
[time,   footPos_x(1,:),footPos_y(1,:)] = textread('footPosLF.txt', format_input2);
[time,   footPos_x(2,:),footPos_y(2,:)] = textread('footPosRF.txt', format_input2);
[time,   footPos_x(3,:),footPos_y(3,:)] = textread('footPosLH.txt', format_input2);
[time,   footPos_x(4,:),footPos_y(4,:)] = textread('footPosRH.txt', format_input2);

swing=[];
[time,swing(1,:)] = textread('swingLF.txt', format_input);
[time,swing(2,:)] = textread('swingRF.txt', format_input);
[time,swing(3,:)] = textread('swingLH.txt', format_input);
[time,swing(4,:)] = textread('swingRH.txt', format_input);

[time,zmp_x] =  textread('zmp_x.txt', format_input);
[time,zmp_y] =  textread('zmp_y.txt', format_input);
[time,com_x] =  textread('com_x.txt', format_input);
[time,com_y] =  textread('com_y.txt', format_input);
[time,com_xd] =  textread('com_xd.txt', format_input);
[time,com_yd] =  textread('com_yd.txt', format_input);
[time,jerk_x] =  textread('jerk_x.txt', format_input);
[time,jerk_y] =  textread('jerk_y.txt', format_input);

[time,viol] = textread('viol.txt', format_input);


[step,footHoldsLF(:,1), footHoldsLF(:,2)] = textread('footHoldsLF.txt', format_input2);
[step,footHoldsRF(:,1), footHoldsRF(:,2)] = textread('footHoldsRF.txt', format_input2);
[step,footHoldsLH(:,1), footHoldsLH(:,2)] = textread('footHoldsLH.txt', format_input2);
[step,footHoldsRH(:,1), footHoldsRH(:,2)] = textread('footHoldsRH.txt', format_input2);

%%
addpath('../')

%%
%STATIC PLOT


figure 
subplot(3,1,1)
%NB these plots are just the feet positions are no longer meaningful only
%in the simple square case
ylabel('zmp x')
plot(time  , footPos_x(1,:),'-bo' );hold on;grid on
plot(time  , footPos_x(2,:),'-ro' )
plot(time  , footPos_x(3,:),'-ko' )
plot(time  , footPos_x(4,:),'-mo' )
plot(time  , zmp_x,'-bo' );hold on;grid on
legend('LF','RF','LH','RH')



subplot(3,1,2)

plot(time  , footPos_y(1,:),'-bo' );hold on;grid on
plot(time  , footPos_y(2,:),'-ro' )
plot(time  , footPos_y(3,:),'-ko' )
plot(time  , footPos_y(4,:),'-mo' )
plot(time  , zmp_y,'-bo' );hold on;grid on

% plot(time  , swing(1,:),'-bo' );hold on;grid on
% plot(time  , swing(2,:),'-ro' )
% plot(time  , swing(3,:),'-ko' )
% plot(time  , swing(4,:),'-mo' )
legend('LF','RF','LH','RH')

subplot(3,1,3)
plot(time, viol,'bo-');grid on

close all
 
f = figure(2)
plot(time, viol,'b');grid on
set(f, 'Paperunits' , 'centimeters', 'PaperSize', [15 12], 'PaperPosition', [0 0 15  12]);
print(f, '-dpdf','../docs/constrainViolationCoupled.pdf')
close(f)

f = figure(3)
plot(time, com_xd, 'b');grid on;hold on 
plot(time, com_yd, 'r');grid on
legend({'x','y'},'interpreter','latex','orientation','vertical','location' ,'southeast','FontSize',20)
ylabel('$\dot{X}$  [$m/s$]','interpreter','latex','FontSize',20)
xlabel('Time  [$s$]','interpreter','latex','FontSize',20)
set(f, 'Paperunits' , 'centimeters', 'PaperSize', [15 12], 'PaperPosition', [0 0 15  12]);
print(f, '-dpdf','../docs/trackingVelocity.pdf')
close(f)

% %%simple test on 1 var
% h2=subplot(2,1,2)
% ylabel('zmp y')
% plot(time  , zmp_y,'-bo' );hold on ; grid on
% plot(time  , com_y,'-k' )
% linkaxes([h1 h2],'x')

%%
close all

%%REPLAY TRAJECTORY
VIDEO = false
if VIDEO
    %video
    aviobj = avifile('video.avi');%default mpeg
    aviobj.Quality = 100;    % Default 75
end

simfig = figure;
subplot(3,1,1)  ; hold on;
for i=1:size(footHoldsLF,1)
    plot(footHoldsLF(i,1), footHoldsLF(i,2), '.b', 'MarkerSize',40)
    plot(footHoldsRF(i,1), footHoldsRF(i,2), '.b', 'MarkerSize',40)
    plot(footHoldsLH(i,1), footHoldsLH(i,2), '.r', 'MarkerSize',40)
    plot(footHoldsRH(i,1), footHoldsRH(i,2), '.r', 'MarkerSize',40)
end

for i=1:length(time)
    stance_vec = {}; 
    stance_count = 0;
    for leg=1:4
        if ~swing(leg,i)
            stance_count = stance_count +1;
            stance_vec{ stance_count} = [footPos_x(leg,i); footPos_y(leg,i);0];
        end
    end
    
    
    subplot(3,1,1)
    h = plotPolygon(stance_vec);   
    hold on
    h(length(h)+2) = plot(com_x(i), com_y(i),'.g','MarkerSize',30);
    h(length(h)+1) = plot(zmp_x(i), zmp_y(i),'.k','MarkerSize',40);

    legend([h(length(h)+1) h(length(h)+2)], 'zmp', 'com','location', 'West')
    xlim([-1 ,2])
    ylim([-1.1 ,2.0])
    
    
    
    subplot(3,1,2)
    plot(time(i), viol(i),'.b','MarkerSize',20); hold on; grid on;
    xlim([0, time(end)])
    ylim([0,1])
    ylabel('constr.viol')
    
    subplot(3,1,3)
    plot(time(i), swing(1,i),'.b','MarkerSize',20); hold on;grid on;
    plot(time(i), swing(2,i),'.r','MarkerSize',20); hold on;grid on;
    plot(time(i), swing(3,i),'.k','MarkerSize',20); hold on;grid on;
    plot(time(i), swing(4,i),'.m','MarkerSize',20); hold on;grid on;
    xlim([0, time(end)])
    ylim([0,1.1])
    ylabel('swing leg')
    
    legend('LF','RF','LH','RH')
    pause(0.001+ 1/time)
    drawnow
    
    if VIDEO
        set (gcf,'Units','Normalized','outerposition',[0 0 1 1]); %do full screen window
        frame = getframe(gcf);
        aviobj = addframe(aviobj,frame);
    end
    delete(h); %this deletes the handle

                  
end   
   
if VIDEO
    aviobj = close(aviobj)
end

%close(simfig)


%to show importance weighting
% figure
% [x,y]= gaussianVector(10,5,2)
% plot(x,y)