
%brief Performs a clockwise radial sort of the input points
%       starting with the bottom left point
% @param[out] p Unsorted points or footholds.
%Uses the slow n^2 sort algorithm, shouldn't matter for sorting 4 points :)
%Fails when 3 points are on same line and one could be removed
% */

function sorted_stancevec =  clockWiseSort(stance_vec)

% make sure the first point is the one with lowest (x,y) value
nlegs = size(stance_vec,2);

% sort clockwise
for i = 2:(nlegs - 1)
    for j= i+1:nlegs
        %the point stance_vec{j} should always be on the right to be cwise thus if it
        %is on the left <0 i swap
        if (Point2isRightOfLine(stance_vec{1}, stance_vec{i}, stance_vec{j})  < 0.0) %for ccwise should be >0
            tmp = stance_vec{i};
            stance_vec{i} = stance_vec{j};
            stance_vec{j} = tmp;
        end
    end
end

sorted_stancevec = stance_vec;
end
