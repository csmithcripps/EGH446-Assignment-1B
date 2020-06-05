function dtransform = distanceTransform(map, goal)
    % Input:
    %   map is an arbitrary size real matrix representing an occupancy grid. The elements are equal to 0 (free space) or 1 (occupied)
    %   goal is a 1x2 matrix containing the end coordinate [x y] for the path
    % Return:
    %   dtransform is a matrix, the same size as map, whose elements reflect the distance from that cell to the goal
    
    % compute the distance transform of map
    [mapy,mapx] = size(map);
    og = zeros([mapy+2 mapx+2]);
    og(1,:)=NaN;
    og(:,1)=NaN;
    og(mapy+2,:)=NaN;
    og(:,mapx+2)=NaN;
    for x=2:mapx+1
        for y=2:mapy+1
            if map(y-1,x-1)==1
                og(y,x)=NaN;
            end
             if map(y-1,x-1)==0
                 og(y,x)=inf;
             end
        end
    end
    og(goal(2)+1,goal(1)+1)=0;
    while any(any(isinf(og)))
    for x=2:mapx+1
        for y=2:mapy+1
            if ~isnan(og(y,x))
            og(y,x)=min(min(og(y-1:y+1,x-1:x+1)+[sqrt(2) 1 sqrt(2);1 0 1;sqrt(2) 1 sqrt(2)]));
            end
        end
    end
    end                
    
    dtransform =og(2:mapy+1,2:mapx+1);
end
