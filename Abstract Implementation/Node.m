%To keep with topological_navigation, this will remain as Node)
classdef Node < handle
    
    properties
        position = [] %rename to pose
%         nodePlot
        name %Pretty name
        prettyName
        id %unique identifier
        neighbour_list = []
        neighbourPrettyName
    end
    
    methods
        function N = Node(prettyName, position)
            N.prettyName = prettyName;
            N.position = position;
        end
    end
    
end























