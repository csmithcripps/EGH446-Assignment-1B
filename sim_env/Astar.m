classdef Astar
    
    methods (Static)

        function newlist = qappend(list, nodeid)
            % add the node id to the end of the list and return an updated list 
            %
            % Input:
            %   nodeid  (scalar)
            %   list    (vector)
            %
            % Output:
            %    newlist (vector)


            newlist = [list nodeid];
        end

        function in = qcontains(list, nodeid)
            % return true if the node id is in the list, otherwise false
            % 
            % Input:
            %   list    (vector)
            %   nodeid  (scalar)
            %
            % Output:
            %   in (logical)

            in = ~isempty(find(list==nodeid,1));
        end

        function [newlist,nodeid] = qpop(list)
            % get a node id from the front of list and return an updated list. If the list is
            % empty return newlist as and nodeid as the empty value [].
            % 
            % Input:
            %   list    (vector)
            %
            % Output:
            %   newlist (vector)
            %   nodeid  (scalar)
            newlist = list(2:end);
            if isempty(list)
                nodeid = [];
            else
                nodeid = list(1);
            end
        end

        function [newlist,nodeid] = qinsert(list, nodeid, cost)
            % insert the node id into the list and return an updated list.  The node is inserted
            % at a location such that the cost of the nodes is non-decreasing in magnitude.
            % cost is a vector such that cost(i) is the cost of node i. It is guaranteed that 
            % cost will be a vector with length at least equal to the maximum nodeid.
            % If multiple nodes have the same cost, their relative ordering does not matter.
            % 
            % 
            % Input:
            %   list    (vector)
            %   nodeid  (scalar)
            %   cost    (vector)
            %
            % Output:
            %   newlist (vector)
            
            nodeCost = cost(nodeid);
            
            for i = 1:length(list)
                if nodeCost < cost(list(i))
                    newlist = [list(1:i-1) nodeid list(i:end)];
                    return;
                end
            end
            newlist = [list nodeid];
        end
        
        function newList = qremove(list,nodeid)
            [~,i] = find(list==nodeid);
            newList = [list(1:i-1) list(i+1:end)];
        end
        
        function nodeId = nodeExists(rowList, colList, curRow, curCol)
            i=1;
            while(rowList(i) ~= curRow || colList(i) ~= curCol )
                i=i+1;
                if(i>length(rowList))
                    nodeId = [];
                    return;
                end
            end
            nodeId = i;
        end
        
        function path = AstarPath(map, startRow, startCol, goalRow, goalCol)
            % Find the most optimal path between two points within an 
            % Accupancy grid
            % 
            % 
            % Input:
            %   map         An arbitrary sized logical occupancy grid
            %   startRow    The row of the start point
            %   startCol    The column of the start point
            %   goalRow     The row of the goal point
            %   goalCol     The column of the goal point
            %
            % Output:
            %   path        A nx2 matrix of the optimal path in occupancy
            %               space
            
            reachedGoal = 0;
            currentNode = 1;
            
            % Store cost of actions
            diagCost = 2;
            actionCost = [diagCost 1 diagCost;...
                          1        0        1;...
                          diagCost 1 diagCost];

            open = [];
            closed = [];
            nodeRow = [];
            nodeCol = [];
            parent = [];
            cost = [];

            % OPEN START NODE    
            nodeRow(currentNode) = startRow;
            nodeCol(currentNode) = startCol;
            parent(currentNode) = 0;
            cost(currentNode) = sqrt((goalRow - startRow)^2 + (goalCol - startCol)^2);
            open = Astar.qinsert(open,currentNode,cost);

            while(~reachedGoal)
                [open,currentNode] = Astar.qpop(open);

                if(isempty(currentNode))
                    reachedGoal = 0;
                    break;
                end

                if (nodeRow(currentNode) == goalRow && nodeCol(currentNode) == goalCol)
                    reachedGoal = 1;
                    break;
                end
                closed = Astar.qappend(closed,currentNode);

                %Open Child nodes
                parentNode = currentNode;
                for i = -1:1
                    for j = -1:1
                        tmpRow = nodeRow(parentNode) + i;
                        tmpCol = nodeCol(parentNode) + j;
                        if map(tmpRow,tmpCol) == 1
                            continue;
                        end
                        tmpCost = cost(parentNode) + actionCost(i+2,j+2) + ...
                            sqrt((goalRow - tmpRow)^2 + (goalCol - tmpCol)^2);
                        
                        childNodeId = Astar.nodeExists(nodeRow,nodeCol,tmpRow,tmpCol);
                        
                        % Check if this is a new Node
                        if(~isempty(childNodeId))
                            % If this node already exists check if it has a
                            % lower cost
                            if cost(childNodeId)>tmpCost
                                if(Astar.qcontains(closed,childNodeId))
                                    % If this node is in the closed list
                                    % move to open with th new cost
                                    closed = Astar.qremove(closed,childNodeId);
                                    parent(childNodeId) = parentNode;
                                    cost(childNodeId) = tmpCost;
                                    open = Astar.qinsert(open,childNodeId,cost);
                                elseif (Astar.qcontains(open,childNodeId))
                                    % If this node is in the open list
                                    % reinsert with the new cost
                                    open = Astar.qremove(open,childNodeId);
                                    parent(childNodeId) = parentNode;
                                    cost(childNodeId) = tmpCost;
                                    open = Astar.qinsert(open,childNodeId,cost);
                                end
                            end   
                        else
                            % if this is a new node save it's location and
                            % cost and add to the open list
                            childNodeId = length(cost) + 1;
                            nodeRow(childNodeId) = tmpRow;
                            nodeCol(childNodeId) = tmpCol;
                            parent(childNodeId) = parentNode;
                            cost(childNodeId) = tmpCost;
                            open = Astar.qinsert(open,childNodeId,cost);
                        end 
                    end
                end   
            end

            path = [];
            while(currentNode ~= 1)
                path = [nodeRow(currentNode) nodeCol(currentNode); path];
                currentNode = parent(currentNode);
            end


        end
        
        function orderedList = WPGraphSearch(wpList, startPos)
            % Find the approximate optimal order of WPs
            % 
            % Input:
            %   wpList          List of waypoints (x y)
            %   startPos        Robot Start Position
            %
            % Output:
            %   orderedList     A sorted list of the previously mentioned
            %                   WPs
            
            
            numWP = size(wpList,1);
            tmpList = [startPos; wpList];
            for i = 1:numWP+1
                for j = 1:numWP+1
                    distMat(i,j) = sqrt((tmpList(i,1) - tmpList(j,1))^2 + ...
                                        (tmpList(i,2) - tmpList(j,2))^2);
                end
            end
            
            distMat(1:end,1) = 0;
            
            options = perms(1:numWP);
            bestPath = 1;
            bestCost = inf;
            for i = 1:length(options)
                currCost = 0;
                currPoint = 1;
                nextPoint = options(i,1)+1;
                for j = 1:numWP
                    currCost = currCost + distMat(currPoint,nextPoint);
                    currPoint = nextPoint;
                    nextPoint = options(i,j)+1;
                end
                if currCost<bestCost
                    bestCost = currCost;
                    bestPath = i;
                end
            end
            orderedList = options(bestPath,:);
        end
    end % static methods
end % classdef