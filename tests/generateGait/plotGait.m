close all
clear all
clc
format short g
format_input = '%f  %f';
format_input2 = '%f  %f %f';


addpath('../')
loadFigOptions
addpath('../../../../../../install/share/crawl_planner/tests/generateGait')
    
[time,   footPos_x(1,:),footPos_y(1,:)] = textread('footPosLF.txt', format_input2);
[time,   footPos_x(2,:),footPos_y(2,:)] = textread('footPosRF.txt', format_input2);
[time,   footPos_x(3,:),footPos_y(3,:)] = textread('footPosLH.txt', format_input2);
[time,   footPos_x(4,:),footPos_y(4,:)] = textread('footPosRH.txt', format_input2);

[time,grForcesZ(1,:)] = textread('grForcesLF_Z.txt', format_input);
[time,grForcesZ(2,:)] = textread('grForcesRF_Z.txt', format_input);
[time,grForcesZ(3,:)] = textread('grForcesLH_Z.txt', format_input);
[time,grForcesZ(4,:)] = textread('grForcesRH_Z.txt', format_input);


[time,   des_target_posX(1,:),des_target_posY(1,:)] = textread('basePosition.txt', format_input2);
[time,   des_target_posXd(1,:),des_target_posYd(1,:)] = textread('baseVelocity.txt', format_input2);


swing=[];
[time,swing(1,:)] = textread('swingLF.txt', format_input);
[time,swing(2,:)] = textread('swingRF.txt', format_input);
[time,swing(3,:)] = textread('swingLH.txt', format_input);
[time,swing(4,:)] = textread('swingRH.txt', format_input);
[time,strideparam] = textread('strideparam.txt', format_input);

%fill in remaining values
grForcesX = zeros(4, length(time));
grForcesY = zeros(4, length(time));
des_target_posZ = 0.55*ones(1, length(time));
footPos_z = zeros(4, length(time));

%STATIC PLOT
figure 
plot(time  , swing(1,:)*0.25,'-bo' );hold on;grid on
plot(time  , swing(2,:)*0.5,'-ro' )
plot(time  , swing(3,:)*0.75,'-ko' )
plot(time  , swing(4,:),'-mo' )
legend('LF','RF','LH','RH')




%%
%plot foot positions
figure 
plot(time  , footPos_x(1,:),'-bo' );hold on;grid on
plot(time  , footPos_x(2,:),'-ro' )
plot(time  , footPos_x(3,:),'-ko' )
plot(time  , footPos_x(4,:),'-mo' )
legend('LF','RF','LH','RH')
figure 
plot(time  , footPos_y(1,:),'-bo' );hold on;grid on
plot(time  , footPos_y(2,:),'-ro' )
plot(time  , footPos_y(3,:),'-ko' )
plot(time  , footPos_y(4,:),'-mo' )
legend('LF','RF','LH','RH')

%plot grforces
figure
plot(time  , grForcesZ(1,:),'-bo' );hold on;grid on
plot(time  , grForcesZ(2,:),'-ro' )
plot(time  , grForcesZ(3,:),'-ko' )
plot(time  , grForcesZ(4,:),'-mo' )
legend('LF','RF','LH','RH')

figure
plot(time,  des_target_posX(1,:))
plot(time,  des_target_posY(1,:))

%
close all
simfig = figure;


for i=1:length(time)
    stance_vec = {}; 
    stance_count = 0;
    % this avoids a bit the slow down due to hold on
    cla
    
    for leg=1:4
        if ~swing(leg,i)
            stance_count = stance_count +1;
            stance_vec{ stance_count} = [footPos_x(leg,i); footPos_y(leg,i);0];
        end
    end
    
    
    subplot(2,1,1)
        
    com = plot(des_target_posX(i), des_target_posY(i),'.r','MarkerSize',40);
    hold on;
    
    h = plotPolygonNiraj(stance_vec);   
    
    hold on
    xlim([-1 ,2])
    ylim([-1.1 ,1.0])
    %plot base pos
   
    subplot(2,1,2)
    hold on;
    %plot only when 1
    plot(time(1:i), swing(1,1:i)*0.25,'.b','MarkerSize',20); grid on; 
    plot(time(1:i), swing(2,1:i)*0.5,'.r','MarkerSize',20); grid on;
    plot(time(1:i), swing(3,1:i)*0.75,'.k','MarkerSize',20); grid on;
    plot(time(1:i), swing(4,1:i),'.m','MarkerSize',20); grid on;
    
    
    xlim([0, time(end)])
    ylim([0.01,1.1])
    ylabel('swing leg')
    legend('LF','RF','LH','RH')

    %
    set(gcf, 'renderer', 'painters')
    drawnow   limitrate
   
    delete(h); %this deletes the handle
    delete(com);

                  
end   
   

