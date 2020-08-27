classdef Network < handle
    
    properties
        node_list = nan  %node_list(1,:) == [nodeName, xpos, ypos]
        nlr              %node_list(nlr(id),:) == node details for id
        edge_list = nan
        adjacency_matrix
        node_locations
        NodeNameList
        r %randomly generated id (for debugging purposes)
    end
    
    methods
        
        function Net = Network(node_locations, NodeNameList, adjacency_matrix)
            Net.r = rand();
            
            %Number of Nodes
            Net.node_locations = node_locations;
            Net.NodeNameList = NodeNameList;
            
            %Adjacency matrix
            Net.adjacency_matrix = adjacency_matrix;
            [x,y] = find(Net.adjacency_matrix);
            aa=[x,y]'; 
            
            
            %Create the nodes
            Net.node_list = Node("", [0,0]);
            nlr = nan(1,max(node_locations(:,1)));
            for i=1:size(node_locations,1)
                nd = node_locations(i,:);
                Net.node_list(i) = Node(NodeNameList(i), nd(2:3));
                Net.node_list(i).name = nd(1);
                Net.node_list(i).id = i;
                nlr(nd(1)) = i;
            end
            Net.nlr = nlr;
            
            %Identify and store the neighbour identifiers
            for i=1:size(node_locations,1)
                nom=Net.node_list(i).name;
                ab=aa(:,any(aa==nom)); ac=ab(:); ac(ac==nom)=[];
                Net.node_list(i).neighbour_list = Net.nlr(ac);
                neigh = Net.node_list(Net.nlr(ac));
                for j=1:length(neigh)
                    Net.node_list(i).neighbourPrettyName(j)=neigh(j).name;
                end
%                 Net.node_list(i).delays = zeros(1,length(neigh));
%                 Net.node_list(i).delayed_time = zeros(1,length(neigh));
            end
%             for N=Net.node_list
%                 ab=aa(:,any(aa==N.name)); ac=ab(:); ac(ac==N.name)=[];
%                 N.neighbour_list = Net.nlr(ac);
%             end
            %neighbourlist is still not generating correctly,
            %O.findNode(76) lists WayPoint43, with neighbour ID 108, but
            %O.findNode(108) lists WayPoint75, not WayPoint106 which is at
            %ID 9 (found using O.findNodeByName(106))
            %43-106 is a pair in O.EER which is generated from the
            %edge_list, so the edges are being generated correctly. Thus
            %the issue lies above.
            
            
            %Create the edges
            Net.edge_list = Edge(Node("",[0,0]), Node("",[0,0]));
            for j = 1:length(x)
                yj=Net.nlr(y(j));
                xj=Net.nlr(x(j));
                Net.edge_list(j) = Edge(Net.node_list(yj),...
                                        Net.node_list(xj));
            end
            
        end
        
        function plot(Net)
            hold on
            for edge = Net.edge_list
               edge.Plot()
            end
            
            
            for node = Net.node_list
                node.Plot();
            end
            hold off
            
            ax=gca;
            axis(ax,'off');
        end
        function quickPlot(Net)
            hold on
            Net.slowPlotEdges();
            Net.quickPlotNodes();
            hold off
            
            set(gca, 'Position', [0.05 0.05 0.9 0.9], 'visible','off');
        end
        function slowPlotEdges(Net)
            for edge = Net.edge_list
               edge.Plot()
            end
        end
        function quickPlotNodes(Net)
            scatter(Net.node_locations(:,2), Net.node_locations(:,3),'k','filled')
        end
        function plotNodeNames(Net)
            hold on
            
            for node = Net.node_list
                pos = node.position;
                text(pos(1)+.3,pos(2)-.3,string(node.id),'Color',[0,0,0]);
            end
            
            hold off
        end
        function plotNodeTimes(Net)
            hold on

            for node = Net.node_list
                %disp(node.time_from)
                
                pos = node.position;
                text(pos(1)-.5, pos(2)-.5, string(node.time_from),...
                    "Color", [0,.7,1]);
            end
            
            hold off
        end
        function joinNodes(Net, nodeID_1, nodeID_2, aid, atotal)
            hold on
            
            n1 = Net.node_locations(nodeID_1,:);
            n2 = Net.node_locations(nodeID_2,:);
            
            p = prism(atotal);
            plot([n1(2), n2(2)], [n1(3), n2(3)], 'Color', p(aid,:), 'LineWidth', 4)
            
            hold off
        end
            
    end
end