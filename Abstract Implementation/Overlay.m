classdef Overlay < handle
    
    properties
        Network
        EdgeExtensions = [];
        EER = []; %EdgeExtensionsReference
        NodeExtensions = [];
        NER = []; %NodeExtensionsReference
        NER_Name = []; %NodeExtensionsReferenceName
    end
    
    methods
        function O = Overlay(Network)
            O.Network = Network;
            
            O.EdgeExtensions = EdgeExt.empty(0,length(Network.edge_list));
            for i = 1:length(Network.edge_list)
                O.EdgeExtensions(i) = EdgeExt(Network.edge_list(i));
            end
            EN1 = [Network.edge_list.node1]';
            EN2 = [Network.edge_list.node2]';
            O.EER = [EN1.id;EN2.id]';
            
            O.NodeExtensions = NodeExt.empty(0,length(Network.node_list));
            for i = 1:length(Network.node_list)
                O.NodeExtensions(i) = NodeExt(Network.node_list(i));
            end
            O.NER = [Network.node_list.id];
            O.NER_Name = [Network.node_list.name];
        end
        
        function overlayCopy = Copy(O)
            
            newNet = Network(...
                O.Network.node_locations, ...
                O.Network.adjacency_matrix); %#ok<CPROP>
            
            overlayCopy = Overlay(newNet);
            
            %Copy reservation information
            for i=1:length(O.EdgeExtensions)
                overlayCopy.EdgeExtensions(i).reservations = ...
                          O.EdgeExtensions(i).reservations;
            end
            
        end
        
        function RemovePath(O, agent_id)
            
            for EExt = O.EdgeExtensions
                
                EExt.PurgeAgent(agent_id);
                
            end
            
        end
        
        function node = findNode(O, nodeID)
            node = O.NodeExtensions(nodeID==O.NER).node;
        end
        function node = findNodeExt(O, nodeID)
            node = O.NodeExtensions(nodeID==O.NER);
            
%             for node2=O.NodeExtensions
%                 if node2.node.id == nodeID
%                     return
%                 end
%             end
        end
        
        function node = findNodeByName(O, nodeName)
            for node = O.Network.node_list
                if node.name == nodeName
                    return
                end
            end
        end
        function node = findNodeExtByName(O, nodeName)
            node = O.NodeExtensions(nodeName==O.NER_Name);
        end
        
        function neighbours = findNeighbours(O,NExt)
            neighbours = {};
            for N_id = NExt.node.neighbour_list
                neighbours{end+1} = O.findNodeExt(N_id);                
            end
            
        end
        
        function edgeRet = findEdgeExt(O, ID1, ID2)
                edgeRet = O.EdgeExtensions(any([all(O.EER'==[ID1;ID2]);...
                                                all(O.EER'==[ID2;ID1])]));
                                            
%                 for edgeRet2=O.EdgeExtensions
%                     if edgeRet2.edge.node1.id == ID1
%                         if edgeRet2.edge.node2.id == ID2
%                             return
%                         end
%                     end
%                     if edgeRet2.edge.node1.id == ID2
%                         if edgeRet2.edge.node2.id == ID1
%                             return
%                         end
%                     end
%                 end
        end
        
    end
end

