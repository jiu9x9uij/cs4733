
function shortest_path = dijkstra(verticies, edges, start, goal)

% verticies is a list of (x, y) pairs
% edges is a list of (vert_idx1, vert_idx2, cost)
% start is vert_idx of start
% goal is vert_idx of goal
% shortest_path is list of vert_idx hops

    % if start and goal are same vertex, done
    if (start == goal)
        shortest_path = start;
        return;
    end

    num_verticies = length(verticies);      % number of verticies
    settled = cell(num_verticies, 1);       % checked nodes
    unsettled = cell(1);                    % remaining nodes
    dist = inf(1, num_verticies);           % distance from start to vertex
    predecessors = zeros(1, num_verticies); % link for recreating path

    % initialize
    dist(start) = 0;
    unsettled{1} = start;
    
    while (~isempty(unsettled))
        
        % get minimum unsettled
        node = -1;
        idx = 0;
        for i=1:length(unsettled)
            vertex = unsettled(i);
            vertex = vertex{1};
            if (node == -1 || dist(vertex) < dist(node))
                node = vertex;
                idx = i;
            end
        end
        
        settled{node} = 1;
        unsettled(idx) = [];
        
        for i=1:length(edges)
            edge = edges(i,:);
            if (edge(1) == node && isempty(settled{edge(2)}))        
                neighbor = edge(2);                
                newDist = dist(node) + edge(3);
                if (dist(neighbor) > newDist)
                    dist(neighbor) = newDist;
                    predecessors(neighbor) = node;
                    unsettled{length(unsettled)+1} = neighbor;
                end
            end
        end
        
    end

    % now get path from start to goal
    if (~predecessors(goal))
        shortest_path = [];
    else
        shortest_path = zeros(1, num_verticies);
        i = 1;
        current = goal;
        while (current)
            shortest_path(i) = current;
            current = predecessors(current);
            i = i+1;
        end
        shortest_path = fliplr(shortest_path(:,1:i-1));
    end
    
end
