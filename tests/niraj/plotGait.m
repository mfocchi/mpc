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
[time,strideparam] = textread('strideparam.txt', format_input);

%%
addpath('../')

%%
%STATIC PLOT
figure 
plot(time  , swing(1,:),'-bo' );hold on;grid on
plot(time  , swing(2,:),'-ro' )
plot(time  , swing(3,:),'-ko' )
plot(time  , swing(4,:),'-mo' )
legend('LF','RF','LH','RH')

%%

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



%%
close all

simfig = figure;


for i=1:length(time)
    stance_vec = {}; 
    stance_count = 0;
    for leg=1:4
        if ~swing(leg,i)
            stance_count = stance_count +1;
            stance_vec{ stance_count} = [footPos_x(leg,i); footPos_y(leg,i);0];
        end
    end
    
    
    subplot(2,1,1)
    h = plotPolygonNiraj(stance_vec);   
    hold on
    xlim([-1 ,2])
    ylim([-1.1 ,1.0])
            
     subplot(2,1,2)
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

    delete(h); %this deletes the handle

                  
end   
   
if VIDEO
    aviobj = close(aviobj)
end

