classdef Edge < handle
    
    properties
        node1
        node2
        weight
        edgePlot
    end
    
    methods
        function E = Edge(node1, node2)
%             disp(node1.id + " - " + node2.id);
            E.node1 = node1;
            E.node2 = node2;
            E.weight = sqrt(power(node1.position(1)-node2.position(1),2)...
                          + power(node1.position(2)-node2.position(2),2));
        end
        
        function Plot(E)
            E.edgePlot = plot([E.node1.position(1), E.node2.position(1)],...
                              [E.node1.position(2), E.node2.position(2)],...
                              'k');
        end
        
        
        function ChangeState(E, state)
            switch (state)
                case "closed"
                    E.edgePlot.Color = [1,0,0];
                case "open"
                    E.edgePlot.Color = [0,1,0];
                case "current"
                    E.edgePlot.Color = [0,0,1];
                case "null"
                    E.edgePlot.Color = [0,0,0];
                case "selected"
                    E.edgePlot.Color = [1,0,1];
            end
            %pause(.2)
        end
        
    end
end