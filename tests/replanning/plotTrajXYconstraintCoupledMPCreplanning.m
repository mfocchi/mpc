close all
clear all
clc
format short g
format_input = '%f  %f ';
format_input2 = '%f  %f %f';

addpath('~')
addpath('../')

[horizon_size, number_of_steps, experiment_duration] = textread('./replan_data/exp_data', format_input2);
replanning_window = horizon_size/number_of_steps;
replanning_stages = experiment_duration/replanning_window;
simfig = figure;
colors= dec2bin(0:1:2^3-1)-'0';

addpath('../') %for plot polygon

dist_sample = 1;
for plan=1:replanning_stages
    disp(plan)
    loadTraj
        
    foothold_handle=[];
    com_handle=[];
    
    %%plot the foothold for this replanning window
    subplot(3,1,1)  ; hold on; grid on
    for j=1:size(footHoldsLF,1)
        foothold_handle(1,j) =plot(footHoldsLF(j,1), footHoldsLF(j,2), '.', 'MarkerSize',40,'color',colors(plan,:));
        foothold_handle(2,j) =plot(footHoldsRF(j,1), footHoldsRF(j,2), '.', 'MarkerSize',40,'color',colors(plan,:));
        foothold_handle(3,j) =plot(footHoldsLH(j,1), footHoldsLH(j,2), 'o', 'MarkerSize',10,'color',colors(plan,:));
        foothold_handle(4,j) =plot(footHoldsRH(j,1), footHoldsRH(j,2), 'o', 'MarkerSize',10,'color',colors(plan,:));
    end
    %check if velocity has been achieved at the end of the replanning window
    [com_xd(replanning_window) com_yd(replanning_window)]
  
    
    for i=1:replanning_window   
        stance_vec = {};
        stance_count = 0;
        swing_handle =[];
        
        for leg=1:4
            if ~swing(leg,i)
                stance_count = stance_count +1;
                stance_vec{ stance_count} = [footPos_x(leg,i); footPos_y(leg,i);0];
            else
               subplot(3,1,1)
               swing_handle = plot(footPos_x(leg,i), footPos_y(leg,i), '.', 'MarkerSize',40,'color',[0.8 0.5 0]);  
            end
        end
        %plot the support polygons
        subplot(3,1,1)
        support_handle = plotPolygon(stance_vec);
        hold on
       
        %plot com zmp traj
        com_handle(2) = plot(com_x(i), com_y(i),'.','MarkerSize',30,'color',[0.3 , 0.3, 0.3]);
        com_handle(1) = plot(zmp_x(i), zmp_y(i),'.','MarkerSize',30,'color',colors(plan,:));
    

        
        legend([   com_handle(1)  com_handle(2)], 'zmp', 'com','location', 'West')
        xlim([-1 ,1])
        ylim([-2 ,3.0])

        
        %plot the jerk disturbance
        subplot(3,1,2)
        plot(dist_sample, jerk_disturbance(dist_sample),'.b','MarkerSize',20); hold on;grid on
        dist_sample = dist_sample+1;
        xlim([0, experiment_duration])
        ylim([-1.1,1.1])
        ylabel('jerk disturbance')
        
        subplot(3,1,3)
        plot(dist_sample, swing(1,i),'.b','MarkerSize',20); hold on;grid on;
        plot(dist_sample, swing(2,i),'.r','MarkerSize',20); hold on;grid on;
        plot(dist_sample, swing(3,i),'.k','MarkerSize',20); hold on;grid on;
        plot(dist_sample, swing(4,i),'.m','MarkerSize',20); hold on;grid on;
        xlim([0, experiment_duration])
        ylim([0,1.1])
        ylabel('swing leg')
        legend('LF','RF','LH','RH')
        pause(0.4)
        drawnow
        
         delete(com_handle); %this deletes the handle
         delete(support_handle);
         delete(swing_handle);
                    
    end
    

     delete(foothold_handle);
end   

%close(simfig)
