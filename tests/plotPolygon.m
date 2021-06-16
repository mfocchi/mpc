
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
    

        for i=1:stance_count
                grid on 
                hold on
                h(i,:) = arrow3(edge_start{i},edge{i},'k'  ); %tail at stance_vec1 magnitude  stance_vec2(i,:)-stance_vec1(i,:)
                xlabel('X');
                ylabel('Y');
                h(i+1) = plot(edge_start{i}(1),stance_vec{1}(2), '.b', 'MarkerSize',40);

                %axis equal
        end   
    end
end


    





















