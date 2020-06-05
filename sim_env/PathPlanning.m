classdef PathPlanning
    
    methods (Static)       
        % this wrapper function allows the assessment code to access your functions

        function M = window(A, x, y) 
        % Input:
        %  A an arbitrary sized real matrix, at least 3x3.
        %  x the x-coordinate (horizontal index) of the centre of the window.
        %  y the y-coordinate (vertical index) of the centre of the window.
        % Return:
        %  M as 3x3 matrix which are the elements of M centered on the element (x,y).
        %
        % Should the window extend beyond the edges of the matrix the function must
        % return an empty matrix [].

            % Pad A with NaNs
            B = padarray(A,[1,1],NaN);

            % Return the 3x3 window
            M = B(y:y+2, x:x+2);
        end

        function B = minwin(A) 
        % Input:
        %  A returns a matrix the same size as A where each element of B is the minimum 
        % of a 3x3 window of A centered at the corresponding element.  Elements of B 
        % that correspond to a window which "falls off the edge" of the matrix A should be set to a value of NaN.

            sizes = size(A)
            B = zeros(sizes)


            for x = 1:sizes(2)
                for y = 1:sizes(1)
                    M = window(A,x,y);
                    if isempty(M)
                        B(y,x) = NaN;
                    else
                        B(y,x) = min(min(M));
                    end
                end
            end
        end

        function next = minval(M)
        % Input:
        %  M is a real 3x3 matrix
        % Return:
        %  next is a 1x2 matrix with elements [x, y] which are the horizontal and vertical coordinates relative to the centre
        %       element, of the smallest element in the matrix.
            assert(~isempty(M));
            [row,col] = find(M == min(min(M)));
            next = fliplr([row(1),col(1)] - [2,2]);
        end



        function dtransform = distanceTransform(map, goal)
            % Input:
            %   map is an arbitrary size real matrix representing an occupancy grid. The elements are equal to 0 (free space) or 1 (occupied)
            %   goal is a 1x2 matrix containing the end coordinate [x y] for the path
            % Return:
            %   dtransform is a matrix, the same size as map, whose elements reflect the distance from that cell to the goal
            costMat = [sqrt(2)   1   sqrt(2);
                             1   0   1;
                       sqrt(2)   1   sqrt(2)];

            % Show map in inf/Nan form
            map2 = map;
            map2(map2==0) = inf;
            map2(map2==1) = NaN;
            map2(goal(2),goal(1)) = 0;
            B = map2;
            prevInfs = inf;
            while(1)
                
                % Break if number of infs stops reducing
                currInfs = sum(B == inf);
                if (currInfs >= prevInfs); break; end
                prevInfs = currInfs;
                
                for x = 1:size(B,2)
                    for y = 1:size(B,1)
                        if isnan(B(y,x)); continue; end
                        M = PathPlanning.window(B,x,y);
                        if isempty(M)
                            B(y,x) = NaN;
                        else
                            B(y,x) = min(min(M + costMat));
                        end
                    end
                end
            end



            % compute the distance transform of map
            dtransform = B;
        end

        function path = findPath(map, start, goal,dtransform)
            % Input:
            %   map is an arbitrary size real matrix representing an occupancy grid. The elements are equal to 0 (free space) or 1 (occupied)
            %   start is a 1x2 matrix containing the start coordinate [x y] for the path
            %   goal is a 1x2 matrix containing the end coordinate [x y] for the path
            % Return:
            %   path is an Nx2 matrix containing the ordered path from start to goal, inclusive of end points.  N depends on the map.
        %     
            if isempty(dtransform)
                dtransform = distanceTransform(map, goal);   % use your own distance transform function
            end
            % compute the best path 
            path = [start];
            current = start;

            while ~(current(1) == goal(1) && current(2) == goal(2))
                M = PathPlanning.window(dtransform, current(1),current(2));
                if isempty(M)
                    disp('fuck')
                else
                    Action = PathPlanning.minval(M);
                    current = current + Action;
                    path = [path; current];
                end
            end
        end

    end
end