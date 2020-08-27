function [NodeList, NodeNameList, AdjacencyMatrix] = ReadTmap(yaml_file)
    yaml_file = char("./configuration_files/"+yaml_file);

    if exist(string(yaml_file) + ".mat", 'file') 
        disp("Loading data from: " + string(yaml_file) + ".mat")
    
        S=load(string(yaml_file) + ".mat");
        NodeList = S.NodeList;
        NodeNameList = S.NodeNameList;
        AdjacencyMatrix = S.AdjacencyMatrix;
        return
    end
    
    disp("Opening File: " + string(yaml_file))
    addpath(genpath('./YAMLMatlab_0.4.3'));

    YamlStruct = ReadYaml(yaml_file);

    disp("Identifying Nodes:")
    NodeList = [];
    NodeNameList = [];
    for i = 1:length(YamlStruct)
        n = YamlStruct{i};

        NodeList = [NodeList;...
            i, ...
            n.node.pose.position.x, ... 
            n.node.pose.position.y ];
        NodeNameList = [NodeNameList, string(n.node.name)];
        
    end

    disp("Identifying Edges:")
    AdjacencyMatrix = zeros(length(YamlStruct));
    for i = 1:length(YamlStruct)
        n = YamlStruct{i};

        for j = 1:length(n.node.edges)
            e = n.node.edges{j};

%             eid = [str2double(n.node.name(9:end)),...
%                    str2double(e.node(9:end))];

            eid = [find(NodeNameList==n.node.name,1),...
                   find(NodeNameList==e.node,1)];

            eid = sort(eid);

            AdjacencyMatrix(eid(1),eid(2)) = 1;

        end

    end

    disp("Saving Gathered Data:")
    save(string(yaml_file)+".mat", 'NodeList', 'NodeNameList', 'AdjacencyMatrix');
end