classdef NodeExt < handle %#ok<*PROPLC>
    
    properties
        node
        nodePlot
        
        %A* Properties
        f_cost = -1 %Score
        g_cost = 0  %Path cost
        h_cost = -1 %Dist to target
        parent_id
        
        CRH
        
        total_neighbours 
        arrival_time = 0 %Previously entry_time
        departure_time = 0
        departure_times = []
    end
    
    methods
        function NExt = NodeExt(node)
            NExt.node = node;
            NExt.total_neighbours = length(NExt.node.neighbour_list);
            NExt.departure_times = zeros(NExt.total_neighbours,1);
        end
        
        function nodeID = id(NExt)
            nodeID = NExt.node.id;
        end
        
        function nodeName = name(NExt)
            nodeName = NExt.node.name;
        end
        
        function Purge(NExt) %Most of these may be unnecessary
            NExt.f_cost = -1;
            NExt.g_cost = 0;
            NExt.h_cost = -1;
            NExt.parent_id = 0;
            
            NExt.arrival_time = 0;
            NExt.departure_time = 0;
            NExt.departure_times = zeros(NExt.total_neighbours,1);
        end
        
        function UpdateCosts(NExt, f_cost, g_cost, h_cost)
            NExt.f_cost = f_cost;
            NExt.g_cost = g_cost;
            NExt.h_cost = h_cost;
        end
        
        function UpdateParent(NExt, parent_id, CRH)
            NExt.parent_id = parent_id;
            NExt.CRH = CRH;
        end
        
        function UpdateTimes(NExt, current, edge)
%             NExt.arrival_time = current.departure_time + edge.time_weight;
%             NExt.departure_time = NExt.arrival_time; 
%             NExt.departure_times(NExt.neighbour_list==current.id) = NExt.arrival_time;

            dpt=current.get_departure_times(NExt.id);
            NExt.arrival_time = dpt + edge.time_weight;
%             NExt.set_departure_times(current.id, NExt.arrival_time);
            NExt.departure_times = ones(NExt.total_neighbours,1)*NExt.arrival_time;
        end
        
        function set_departure_times(NExt, neighbourID, time)
            NExt.departure_times(NExt.node.neighbour_list==neighbourID) = time;
        end
        
        function time = get_departure_times(NExt, neighbourID)
            time = NExt.departure_times(NExt.node.neighbour_list==neighbourID);
        end
        
        function setDelay(NExt, delay)
            NExt.departure_time = NExt.arrival_time + delay;
            NExt.set_departure_times(current.id, NExt.arrival_time + delay);
%             NExt.departure_times(NExt.neighbour_list==current.id) = NExt.arrival_time + delay;
        end
            
        function Plot(NExt)
            NExt.nodePlot = scatter(NExt.node.position(1), ...
                                    NExt.node.position(2),'k','filled');
        end
        function ChangeState(NExt, state)
            try
                switch (state)
                    case "closed"
                        NExt.nodePlot.MarkerFaceColor = [1,0,0];
                    case "open"
                        NExt.nodePlot.MarkerFaceColor = [0,1,0];
                    case "current"
                        NExt.nodePlot.MarkerFaceColor = [0,0,1];
                    case "null"
                        NExt.nodePlot.MarkerFaceColor = [0,0,0];
                    case "start"
                        NExt.nodePlot.MarkerFaceColor = [1,.5,0];
                    case "target"
                        NExt.nodePlot.MarkerFaceColor = [1,.5,0];
                    case "selected"
                        NExt.nodePlot.MarkerFaceColor = [1,0,1];
                end
            catch
                disp("Invalid or deleted object?")
            end
            %pause(.2)
        end
        
    end
end

