%% closestToGoal(points, goal): Closest Point to Goal with respect to 8 directions 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 10/14/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  U - potential field
%  points - point [x1 y1; x2 y2; ...]
%  goal - point [x y]

function [point] = closestToGoal(points, goal)
    tempPoint = points(1,:);
    minDst = euclidean_distance(tempPoint(1), tempPoint(2), goal(1), goal(2));
    for i = 1:size(points)
        tempdst = euclidean_distance(points(i, 1), points(i, 2), goal(1), goal(2));
        if(tempdst < minDst)
            tempPoint = points(i, :);
            minDst = tempDst;
        end
    end
    point = tempPoint;
end

