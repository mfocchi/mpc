
function  h = plotPolygon(stance_vec)
    
    stance_vec = clockWiseSort(stance_vec);
    stance_count = length(stance_vec);
      
    h = [];
    if (stance_count>2)
        if  stance_count ==4
            %clockwise 
            edge{1} = stance_vec{2}-stance_vec{1}; edge_start{1} = stance_vec{1}; 
            edge{2} = stance_vec{3}-stance_vec{2}; edge_start{2} = stance_vec{2};
            edge{3} = stance_vec{4}-stance_vec{3}; edge_start{3} = stance_vec{3};
            edge{4} = stance_vec{1}-stance_vec{4}; edge_start{4} = stance_vec{4};        

        else 
            %clockwise 
            edge{1} = stance_vec{2}-stance_vec{1}; edge_start{1} = stance_vec{1}; 
            edge{2} = stance_vec{3}-stance_vec{2}; edge_start{2} = stance_vec{2};
            edge{3} = stance_vec{1}-stance_vec{3}; edge_start{3} = stance_vec{3};

        end
        
        %plot edges

        for i=1:stance_count
                grid on 
                hold on
                h(i,:) = arrow3(edge_start{i},edge{i},'k'  ); %tail at stance_vec1 magnitude  stance_vec2(i,:)-stance_vec1(i,:)
                xlabel('X')
                ylabel('Y')
               
                %axis equal
        end   
        %plot feet
       for i=1:stance_count
                grid on 
                hold on
                h(stance_count+i,:) = plot(edge_start{i}(1),edge_start{i}(2), '.b', 'MarkerSize',40);
                xlabel('X')
                ylabel('Y')               
                %axis equal
        end  
         
    elseif  (stance_count == 2)
        %2 legs in stance
        grid on
        hold on
        h = arrow3(stance_vec{1},stance_vec{2}-stance_vec{1},'k'); %tail at stance_vec1 magnitude  stance_vec2(i,:)-stance_vec1(i,:)
        h(2) = plot(stance_vec{1}(1),stance_vec{1}(2), '.b', 'MarkerSize',40);
        h(3) = plot(stance_vec{2}(1),stance_vec{2}(2), '.b', 'MarkerSize',40);
        xlabel('X')
        ylabel('Y')
    elseif (stance_count == 1)
        %1 stance
        h = plot(stance_vec{1}(1),stance_vec{1}(2), '.b', 'MarkerSize',40);
    end   
end


    





















