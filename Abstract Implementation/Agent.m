classdef Agent < handle
    
    properties
        agent_id
        ROS
        Network
        Overlay
        
        %Movement Information
        startID
        currentID %Name of the current Node of the Agent
        targetID
        targetNumber
        start_time
        idle
        reservations
        reservation_uuid
        planningType
        
        %Heuristic Meta
        heuristic_details
        use_continuous_assignment
        use_dynamic_scoring
        use_context_dependent_heuristics
        independent_heuristic
        dependent_heuristic
        include_irritation
        
        % Context-Independent | Static | Heuristic Data
        direct_euclidian_distance
        optimal_route_length
        optimal_planning_time
        random_score
        irritation
        
        % Context-Dependent | Static | Heuristic Data
        task_details
        agentType
        taskType
        load
        randomCropNode1
        randomCropNode2
        row_to_monitor
        randomRowNode1
        randomRowNode2
        task_importance
        
        %Evluation Metrics
        optimal_arrival_time
        total_replans
        total_deadlocks_overcome
    end
    
    methods
        function A = Agent(ROSSim, currentName, Network)
            A.ROS = ROSSim;
            A.Network = Network; %This should be constructed here
            A.Overlay = A.DownloadOverlay();
            A.currentID = A.Overlay.findNodeExt(currentName).id;
            A.reservation_uuid = 0;
            A.targetNumber = 0;
            A.idle = 1;
            A.total_replans = 0;
            A.optimal_arrival_time = 0;
            A.total_deadlocks_overcome = 0;
            A.start_time = 0;
        end
        function SetupHeuristics(A, heuristic_details)
            A.heuristic_details = heuristic_details;
            
            A.use_continuous_assignment = heuristic_details.use_continuous_assignment;
            A.use_dynamic_scoring = heuristic_details.use_dynamic_scoring;
            A.use_context_dependent_heuristics = heuristic_details.use_context_dependent_heuristics;
            
            A.independent_heuristic = heuristic_details.independent_heuristic;
            A.dependent_heuristic = heuristic_details.dependent_heuristic;
            
            A.include_irritation = heuristic_details.include_irritation;
        end
        function SetupTask(A, agent_type)
            A.agentType = agent_type;
            
            if A.agentType == "logistics"
                A.taskType = "move_to_picker";
                A.load = 0;
                A.task_importance = 3;
            elseif A.agentType == "crop_monitoring"
                A.taskType = "move_to_edge";
                A.task_importance = 2;
            elseif A.agentType == "row_monitoring"
                A.taskType = "move_to_row_start";
                A.task_importance = 1;
            end
            
        end
        
        %% Planning Scripts
        function NavigateTo_static(A, targetID, start_time)
            
            %Identify start and target nodes
            A.start_time = start_time;
            A.startID = A.currentID;
            A.targetID = targetID;
            A.targetNumber = A.targetNumber + 1;
            
            
            %Return if already at target node
            if A.targetID == A.startID
                A.reservations = Reservation(...
                                 A.startID, A.targetID, ...
                                 start_time, start_time, ...
                                 A.agent_id, 0, 1);
                A.reservations.delayed = 0;
                return
            end
            
            
            %Identify planning will commence
            A.idle = false;
            A.planningType = "initial planning.";
            
            
            %Download latest map
            NewOverlay = A.DownloadOverlay();
            Start = NewOverlay.findNodeExt(A.startID);
            Target = NewOverlay.findNodeExt(A.targetID);
            
            
            %Initialise heuristic information
            A.irritation = 0;
            A.total_replans = 0;
            optimal_planning_timer = tic;
            A.optimal_route_length = A.OptimalRoute(A.currentID,A.targetID);
            A.optimal_planning_time = toc(optimal_planning_timer);
            A.optimal_arrival_time = A.optimal_route_length + A.start_time;
            A.random_score = rand();
            A.direct_euclidian_distance = A.pythag(Start.node, Target.node);
            A.total_deadlocks_overcome = 0;
            
            %Identify route
            disp(newline+"Agent "+A.agent_id+" begins planning to node "...
                +Target.name+"n ("+Target.id+"i)");
            [Reservation_List, NewOverlay] = A.astar_modified(...
                A.currentID, A.targetID, NewOverlay, start_time, true);
            A.reservations = Reservation_List;
            A.Overlay = NewOverlay;
            
            %Publish reservation details
%             A.showReservationDetails(Reservation_List)
            A.ReserveEdges(Reservation_List);
        end
        function Replan(A, failed_reservations)
            %Takes list of reservation objects beaten in battle.
            disp(newline+"Agent " + A.agent_id + " begins replanning.")
            
            %Infinite loop detector
            if length(dbstack) > (length(A.ROS.Agent_List)*150)
                disp("we in trouble... long dbstack")
                bp=psuedo_breakpoint; % <- breakpoint
            end
            
            % Update Heuristic information
            A.irritation = A.irritation + 1;
            A.total_replans = A.total_replans + 1;
            
            
            %% Identify Start Node [sN is node at start of conflict edge]
            % Find first path component reached
            failed_reservation_mat=[failed_reservations{:}];
            failed_reservation_mat([failed_reservation_mat.uuid]~=A.reservation_uuid)=[];
            early = min([failed_reservation_mat.position]);
            
            if length(failed_reservations) ~= length(failed_reservation_mat)
                disp("")
            end
            
            if ~isempty(failed_reservation_mat)
                disp("Reservations Valid")
                
                %Find associated reservation
                InitialPath = A.reservations;
                if early~=1
                    sN = InitialPath(early).fromID;
                else
                    sN = A.currentID;
                end
                IPe = InitialPath(early);

            end
            
            %% Replan Paths
            
            if ~isempty(failed_reservation_mat)
            
                % Plan Path from Conflict
                %TODO: add condition to only do if (sN ~= A.currentID)
                [ReservationList1, Overlay1] = A.replan_from_conflict(...
                    sN, IPe, InitialPath, early);
           
            
                % Plan Path from Conflict (with delay)
                [ReservationList2, Overlay2, delay] = A.replan_with_delay(...
                    sN, IPe, InitialPath, early);

            end
                
            % Plan Path from Start
            [ReservationList3, Overlay3] = A.replan_from_start();

            
            %% Use lesser path length
            
            if isempty(failed_reservation_mat)
                times = [1, 1, 0];
            else
                times = [ReservationList1(1).time_out, ...
                     ReservationList2(1).time_out, ...
                     ReservationList3(1).time_out];
            end
            
            
%             times = [ReservationList1(1).time_out, ...
%                      ReservationList2(1).time_out, ...
%                      ReservationList3(1).time_out];
            [~,smallest]=min(times);
            
            
            if smallest == 1
                disp("Agent "+A.agent_id+" replanning used replan from conflict.")
                A.Overlay = Overlay1;
                A.reservations = ReservationList1;
            elseif smallest == 2
                disp("Agent "+A.agent_id+" replanning used replan from conflict (with delay of "+delay+").")
                disp("Agent "+A.agent_id+" including delay on edge "+IPe.fromID+"-"+IPe.toID+" of "+delay+"t.");
                A.Overlay = Overlay2;
                A.reservations = ReservationList2;
            elseif smallest == 3
                disp("Agent "+A.agent_id+" replanning used replan from start.")
                A.Overlay = Overlay3;
                A.reservations = ReservationList3;
            end
            
            A.findTimeIssues(A.reservations)
            
            %% Reserve The Edges on the Path
            A.ReserveEdges(A.reservations);
            
        end
        function score = OptimalRoute(A, startID, targetID)
            overlay = A.InitialiseOverlay();
            A.astar_modified(startID, targetID, overlay, 0, false);
            score = overlay.findNodeExt(targetID).arrival_time;
        end
        
        %% Replan Functions
        function [ReservationList, Overlay] = replan_from_conflict(A, ...
                sN, IPe, InitialPath, early)
            A.planningType = "replan from conflict.";
            A.ddisp("Agent "+A.agent_id+" "+A.planningType)
            
            %Define new Overlay
            Overlay = A.DownloadOverlay();
            
            %Plan new section of route
            [reservation_list, Overlay] = A.astar_modified(...
                sN, A.targetID, Overlay, IPe.time_in, true);
            
            %Append new route section to old section
            ReservationList = [InitialPath(1:early-1), reservation_list];
            for i = 1:length(ReservationList)
                ReservationList(i).position = i;
            end
            A.findTimeIssues(ReservationList)
%             A.showReservationDetails(ReservationList1)
        
        end
        function [ReservationList, Overlay, delay] = replan_with_delay(A, ...
                sN, IPe, InitialPath, early)
            
            A.planningType = "replan with delay.";
            A.ddisp("Agent "+A.agent_id+" "+A.planningType)
            
            %Define new Overlay
            Overlay = A.DownloadOverlay();
            
            %Identify period to delay for
            EExt=Overlay.findEdgeExt(IPe.fromID, IPe.toID);
            timeFrom = EExt.FindAvailablePeriod(IPe.time_in, IPe.time_out);
            delay = timeFrom - IPe.time_in;
            
            %Plan new section of route
            [reservation_list, Overlay] = A.astar_modified(...
                sN, A.targetID, Overlay, timeFrom, true);
            if early > 1
                InitialPath(early-1).delayed = true;
            end
            
            %Append new route section to old section
            ReservationList = [InitialPath(1:early-1), reservation_list];
            
            %
            if isempty(InitialPath(1:early-1))
                r = Reservation(reservation_list(1).fromID, ...
                                reservation_list(1).fromID, ...
                                A.start_time, ...
                                reservation_list(1).time_in, ...
                                A.agent_id, 0, 1);
                r.delayed = true;
                ReservationList = [r,ReservationList];
            end
            
            %Update Positions
            for i = 1:length(ReservationList)
                ReservationList(i).position = i;
            end
            A.findTimeIssues(ReservationList)
%             A.showReservationDetails(ReservationList2)
            
        end
        function [ReservationList, Overlay] = replan_from_start(A)
            A.planningType = "replan from start.";
            A.ddisp("Agent "+A.agent_id+" "+A.planningType)
            
            %Define new Overlay
            Overlay = A.DownloadOverlay();
            
            %Replan from start
            [ReservationList, Overlay] = A.astar_modified(...
                A.currentID, A.targetID, Overlay, A.start_time, true);
            A.findTimeIssues(ReservationList)
%             A.showReservationDetails(ReservationList3)
        end
        
        
        
        %% Overlay Management
        function overlay = InitialiseOverlay(A)
            %Create an fresh empty Overlay
            overlay = Overlay(A.Network); %#ok<CPROP>
        end
        function overlay = DownloadOverlay(A)
            
            %Initialise Overlay object
            overlay = A.InitialiseOverlay();
            
            %Fill Overlay with reservation information
            for i = 1:length(overlay.EdgeExtensions)
                overlay.EdgeExtensions(i).reservations = ...
                A.ROS.Coordinator.Overlay.EdgeExtensions(i).reservations;
            end
            
            %Omit reservation details regarding self
            overlay.RemovePath(A.agent_id);
        end

        %% Motion Planning Script
        function [reservation_list, overlay] = astar_modified(...
                A, startID, targetID, overlay, start_time, generateScores)

            startNodeExt =  overlay.findNodeExt(startID);
            targetNodeExt = overlay.findNodeExt(targetID);
            
            OPEN = [];
            CLOSED = [];
            FAILED = [];
            
            %Add the start node extension to the open list.
            OPEN = [OPEN, startNodeExt];
            
            %Define the time the agent will begin moving.
            startNodeExt.arrival_time = start_time;
            startNodeExt.departure_time = start_time;
            startNodeExt.departure_times = ones(startNodeExt.total_neighbours,1)*start_time;
            
            %Define end point for path generation
            startNodeExt.parent_id = -1;
            xxx=0;
            while 1
                xxx=xxx+1;
                
                %Find smallest edge.
                if ~isempty(OPEN)
                    currentNodeExt = A.findMin(OPEN);
                    A.ddisp(xxx+"        Node "+ currentNodeExt.name+"n in focus.")
                else
                    [OPEN, FAILED] = A.findMinFailed(FAILED, CLOSED, OPEN, targetID, overlay);
                    continue
                end
                
                %Move currentNode from OPEN to CLOSED
                OPEN(OPEN==currentNodeExt)=[];
                CLOSED = [CLOSED, currentNodeExt.id];
                A.ddisp(xxx+"            Node "+ currentNodeExt.name+"n moved from OPEN to CLOSED.")
                    
                %If current point is at target, break.
                if currentNodeExt.id == targetNodeExt.id
                    A.ddisp(xxx+"            Node "+ currentNodeExt.name+"n is at target Node.")
                    reservation_list = A.backprop_path_modified(targetNodeExt, overlay);
                    A.findTimeIssues(reservation_list)
                    return
                end
                
                %Determine costs for each neighbour to the current node.
                neighbour_list = overlay.findNeighbours(currentNodeExt);
                for neighbour_entry= neighbour_list %add {:}
                    neighbourNExt = neighbour_entry{1};
                    A.ddisp(xxx+"            Neighbour " +neighbourNExt.name+ "n is in focus.")
                    
                    EExt=overlay.findEdgeExt(currentNodeExt.id, neighbourNExt.id);
                    
                    %Early Exit
                    if any(neighbourNExt.id == CLOSED)
                        A.ddisp(xxx+"                Neighbour " +neighbourNExt.name+ "n is already CLOSED.")
                        continue 
                    end
                    
                    %If edge taken and CRH battle unsuccessful, skip edge.
                    if generateScores
                        [available, CRH] = A.CRH_BATTLE(currentNodeExt, neighbourNExt, targetNodeExt, EExt);
                    else
                        [available, CRH] = deal(1);
                    end
                    
                    if ~available
                        A.ddisp(xxx+"                Edge "+currentNodeExt.name+"n-"+neighbourNExt.name+"n is NOT available (taken and CRH cant win)");
                        
                        %Edge is unavailable
                        if neighbourNExt.f_cost == -1 % <-- issue here? Inf second time doesnt show best parent?
                            A.ddisp(xxx+"                Edge "+currentNodeExt.name+"n-"+neighbourNExt.name+"n is untouched");
                            
                            %Update details
                            h_cost = A.pythag(neighbourNExt.node, targetNodeExt.node);
                            neighbourNExt.UpdateCosts(Inf, Inf, h_cost);
                            neighbourNExt.UpdateParent(currentNodeExt.id, CRH);
                            neighbourNExt.UpdateTimes(currentNodeExt, EExt);
                            
                            A.ddisp(xxx+"                Parent of "+neighbourNExt.name+"n set to "+currentNodeExt.name+"n");  
                            A.ddisp(xxx+"                Arrival for neighbour "+neighbourNExt.name+"n @ "+neighbourNExt.arrival_time);
                            A.ddisp(xxx+"                Departure for neighbour "+neighbourNExt.name+"n @ "+neighbourNExt.departure_time);
                            
                            %Add failed reservation details to list
                            FAILED = [FAILED, [neighbourNExt.id; currentNodeExt.id; currentNodeExt.f_cost]];
                            A.ddisp(xxx+"                Edge "+currentNodeExt.name+"n-"+neighbourNExt.name+"n added to FAILED");
                        end
                        continue
                    else
                        A.ddisp(xxx+"                Edge "+currentNodeExt.name+"n-"+neighbourNExt.name+"n is up for grabs (empty or CRH can win)");
                    end %ADDITIONAL COMPONENT
                    
                    
                    
                    %Calculate distance travelled and distance to target.
                    g_cost = currentNodeExt.g_cost + EExt.edge.weight;
                    h_cost = A.pythag(neighbourNExt.node, targetNodeExt.node);
                    f_cost = g_cost + h_cost;
                    A.ddisp(xxx+"                Neighbour "+neighbourNExt.name+"n has f_cost of "+round(f_cost,3));
                    
                    %If new path is better, update cost and parent.
                    if any([f_cost<neighbourNExt.f_cost,~any(neighbourNExt==OPEN)])
                        
                        neighbourNExt.UpdateCosts(f_cost, g_cost, h_cost);
                        neighbourNExt.UpdateParent(currentNodeExt.id, CRH);
                        neighbourNExt.UpdateTimes(currentNodeExt, EExt);
                        
                        A.ddisp(xxx+"                Parent of "+neighbourNExt.name+"n set to "+currentNodeExt.name+"n");
                        A.ddisp(xxx+"                Arrival for neighbour "+neighbourNExt.name+"n @ "+neighbourNExt.arrival_time);
                        A.ddisp(xxx+"                Departure for neighbour "+neighbourNExt.name+"n @ "+neighbourNExt.departure_time);
                        
                        %Include node in search for smaller routes.
                        if ~any(neighbourNExt == OPEN)
                            A.ddisp(xxx+"                Neighbour "+ neighbourNExt.name+"n moved to OPEN")
                            OPEN = [OPEN, neighbourNExt];
                        end
                    end
                end
            end
        end
        
        %% CRH Functions
        function [success, CRH_Score] = CRH_BATTLE(A, currentNExt, neighbourNExt, targetNExt, EExt)
            %Return 1 if CRH edge is up for grabs (empty or CRH can win)
            %Return 0 if edge is unavailable (taken and CRH cant win)
           
            
            %Calculate time for use of edge.
            from_time = currentNExt.arrival_time; %Use arrival since the delay is not created yet 
            time_till = currentNExt.arrival_time+EExt.time_weight; %The neighbour arrival_time is not set yet
            A.ddisp("                  Edge "+currentNExt.name+"n-"+neighbourNExt.name+"n wanted from "+from_time+"->"+time_till)
            
            %Check if edge is free at time required (no use for EExt.IsEmpty2)
            conflicts_list = EExt.IsEmpty(from_time, time_till);
            
            %Calculate the CRH score to battle with.
            CRH_Score = A.getCRH(currentNExt, neighbourNExt, targetNExt, EExt);
            
            %Succeed on edge has no conflicts
            if isempty(conflicts_list)
                success = 1;
                return;
            end
            
            %Extract the local CRH score
            conflicting_CRH = 0;
            for R = conflicts_list
                conflicting_CRH = conflicting_CRH + R.CRH;
            end
            
            %Fail on Maximum CRH met
            if conflicting_CRH > 150 %This must be normalised
                success = 0;
                return;
            end
            
            %If local CRH score is smaller, return failure.
            success = (CRH_Score > conflicting_CRH);
            if success
                for R = conflicts_list
                    A.ddisp("Agent " + A.agent_id + " can take edge " + ...
                        EExt.edge.node1.id + "-"+ EExt.edge.node2.id + ...
                        " from Agent " + R.agent_id + " with CRH " + CRH_Score + " using " + A.planningType)
                end
            end
        end
        function CRH = getCRH(A, C, N, T, E) %#ok<*INUSL,*NASGU>
            
            %Instantiate empty variable for storing the score.
            CRH = 0;
            
            if A.use_dynamic_scoring
                %Calculate dynamic CRH score
                CRH = A.GenerateDynamicCRH(C, N, T, E);
            else
                %Calculate static CRH score
                CRH = A.GenerateStaticCRH(C, N, T, E);
            end

            %If including irritation, increment CRH by its value
            if A.include_irritation
                CRH = CRH + A.irritation;
            end
            
            if A.use_context_dependent_heuristics
                CRH = CRH * A.GenerateDependentCRH;
            end
            
        end
        function CRH = GenerateDependentCRH(A)
            CRH = A.task_importance;
        end
        function CRH = GenerateDynamicCRH(A, C, N, T, ~) %#ok<*INUSL,*NASGU>
            CRH = 0;
            
            %Identify independent heuristic
            if any(A.independent_heuristic == "euclidian_distance")
                CRH = CRH + A.pythag(N.node, T.node);
            end
            
            if any(A.independent_heuristic  == "optimal_route_length")
                CRH = CRH + A.OptimalRoute(C.id,T.id);
            end
            
            if any(A.independent_heuristic == "random")
                CRH = CRH + rand();
            end
            
            if any(A.independent_heuristic == "planning_time")
                if (CRH==0); CRH=1; end
                tic; A.OptimalRoute(C.id,T.id);
                CRH = CRH * toc;
            end
            
            if any(A.independent_heuristic == "agent_id")
                CRH = CRH + A.agent_id;
            end
            
            if any(A.independent_heuristic == "no_replanning")
                CRH = CRH + 0;
            end
            
            %Inverse Variants
            if any(A.independent_heuristic == "inverse_euclidian_distance")
                CRH = CRH - A.pythag(N.node, T.node);
            end
            
            if any(A.independent_heuristic  == "inverse_optimal_route_length")
                CRH = CRH - A.OptimalRoute(C.id,T.id);
            end
            
            if any(A.independent_heuristic == "inverse_planning_time")
                tic; A.OptimalRoute(C.id,T.id);
                CRH = CRH - toc;
            end
            
        end
        function CRH = GenerateStaticCRH(A, ~, ~, ~, ~) %#ok<*INUSL,*NASGU>
            CRH = 0;
            
            %Identify independent heuristic
            if any(A.independent_heuristic == "euclidian_distance")
                CRH = CRH + A.direct_euclidian_distance;
            end
            
            if any(A.independent_heuristic  == "optimal_route_length")
                CRH = CRH + A.optimal_route_length;
            end
            
            if any(A.independent_heuristic == "random")
                CRH = CRH + A.random_score;
            end
            
            if any(A.independent_heuristic == "planning_time")
                if (CRH==0); CRH=1; end
                CRH = CRH * A.optimal_planning_time;
            end
            
            if any(A.independent_heuristic == "agent_id")
                CRH = CRH + A.agent_id;
            end 
            
            if any(A.independent_heuristic == "no_replanning")
                CRH = CRH + 0;
            end
            
            
            %Inverse Variants
            if any(A.independent_heuristic == "inverse_euclidian_distance")
                CRH = CRH - A.direct_euclidian_distance;
            end
            
            if any(A.independent_heuristic  == "inverse_optimal_route_length")
                CRH = CRH - A.optimal_route_length;
            end
            
            if any(A.independent_heuristic == "inverse_planning_time")
                CRH = CRH - A.optimal_planning_time;
            end
            
        end
        
        %% Motion Planning Tools
        function minNode = findMin(~, OPEN)
            [~,idx] = min([OPEN.f_cost]);
            minNode = OPEN(idx);
        end
        function [OPEN, FAILED] = findMinFailed(A, FAILED, CLOSED, OPEN, ~, overlay)
           
            A.ddisp("          OPEN List Empty");
            A.ddisp("          FAILED List:")
            A.ddisp(round(FAILED',3))

            FAILED(:,any(FAILED(1,:)==CLOSED'))=[];

            smallest = [0,0,0,0,Inf];
            minimal = FAILED(:,FAILED(3,:)==min(FAILED(3,:)));

            T = overlay.findNodeExt(A.targetID);
            for m = minimal %[neighbour_id; current_id; current_f_cost]

                %Identify nodes
                N = overlay.findNodeExt(m(1)); %neighbour Node
                P = overlay.findNodeExt(m(2)); %parent Node

                %Calculate costs
                EExt = overlay.findEdgeExt(P.id, N.id);
                g_cost = P.g_cost + EExt.edge.weight;
                h_cost = A.pythag(N.node, T.node);
                f_cost = g_cost + h_cost;

                %Idenfity smallest f_cost
                if f_cost < smallest(5) %Swap this out to just append, and find smallest later on
                    smallest=[N.id, P.id, g_cost, h_cost, f_cost];
                end

            end

            %Identify nodes
            N = overlay.findNodeExt(smallest(1));
            P = overlay.findNodeExt(smallest(2));

            %Identify delay time and new arrival time
            EExt = overlay.findEdgeExt(P.id, N.id);
            PArrival = P.arrival_time;
            NArrival = PArrival + EExt.time_weight; %this must be calculated
            NewPDeparture = EExt.FindAvailablePeriod(PArrival, NArrival);
            PDepartureDelay = NewPDeparture - PArrival;

            %Update deadlock count
            EExt.deadlock_overcome = true;
            
            %Save new meta info
            N.UpdateCosts(smallest(3), smallest(4), smallest(5));
            P.departure_time = NewPDeparture;
            P.set_departure_times(N.id, NewPDeparture);
            N.UpdateTimes(P, EExt);

            %Log out some information
            A.ddisp(EExt)
            A.ddisp(EExt.reservations)
            A.ddisp("Agent "+A.agent_id+" including local delay on edge "+...
                P.name+"-"+N.name+" of "+PDepartureDelay+"t.");
            A.ddisp("Agent "+A.agent_id+" entering "+P.name+"n-"+N.name+"n @ t"+P.departure_time);
            A.ddisp("Agent "+A.agent_id+"  exiting "+P.name+"n-"+N.name+"n @ t"+N.arrival_time);

            %Append neighbour to OPEN list
            A.ddisp("            FAILED Node "+ N.name+" moved to OPEN.")
            OPEN = [OPEN, N];

            %Remove neighbour from FAILED list
            A.ddisp("            Edge "+FAILED(1)+"i-"+FAILED(2)+"i removed from FAILED");
            FAILED(:,all(FAILED(1:2,:)==smallest(1:2)')) = [];
            
        end
        
        function dist = pythag(~, nodeA, nodeB)
            Ax = nodeA.position(1);
            Ay = nodeA.position(2);
            Bx = nodeB.position(1);
            By = nodeB.position(2);
            dist = sqrt(power(Ax-Bx,2)+power(Ay-By,2));
        end
        function Reservation_List = backprop_path_modified(A, endpoint, overlay)
            %TODO: swap this out to make the reservation list incremental
            %for position
            current = endpoint;
            path = current;
            
            %Generate list of nodes from endpoint to startpoint
            while current.parent_id ~= -1
                
                %Append new path component
                current = overlay.findNodeExt(current.parent_id);
                path(end+1) = current;
                
            end
            
            %Generate list of Reservation Objects to publish
            %TODO: find a way to not need the following line... {}?
            Reservation_List = Reservation(0,0,0,0,0,0,0);
            for i = 2:length(path)
                Start=path(i);
                Target=path(i-1);
                
                %Create reservation objects
                Reservation_List(i-1) = Reservation(...
                                 Start.id, ...
                                 Target.id, ...
                                 Start.get_departure_times(Target.id), ...
                                 Target.arrival_time, ...
                                 A.agent_id, ...
                                 Target.CRH, ... %A.getCRH(), ...
                                 (length(path)-i)+1);
                Reservation_List(i-1).delayed = Start.arrival_time ~= Start.get_departure_times(Target.id); %Start.departure_time;
            end
            Reservation_List = flip(Reservation_List);
            
        end
        function ReserveEdges(A, reservation_list)
            A.findTimeIssues(reservation_list)
            
            %Generate and apply uuid to the reservation objects
            A.reservation_uuid = A.reservation_uuid + 1;
            for r = reservation_list
                r.uuid = A.reservation_uuid;
            end
            
            %Count deadlocks overcome
            for i = 2:length(reservation_list)
                r=reservation_list(i);
                if r.fromID ~= r.toID
                    EExt = A.Overlay.findEdgeExt(r.fromID, r.toID);
                    A.total_deadlocks_overcome=A.total_deadlocks_overcome...
                        +EExt.deadlock_overcome;
                end
            end
            
            
            %Publish the list of Reservation objects
            A.ROS.Topic_publishReservations(reservation_list);
        end
        
        %% Display Functions
        function plotRouteTimes(A, reservation_list)
            
            %If agent has not been given or has reached its goal, return
            if isempty(reservation_list), return, end
            
            %Define Colour Pallette
            shadeG = linspace(.8, .4, length(reservation_list))';
            shadeR = flip(shadeG);
            
            %For each node after the first, print the time
            for i = 1:length(reservation_list)-1
                Res = reservation_list(i);
                
                %Identify location to print text
                pos=A.Overlay.findNodeExt(Res.toID).node.position-.5;
                
                %Define colour to print times
                colour=[shadeR(i), shadeG(i),0];
                if Res.delayed, colour=[1,0,.7]; end
                
                %Print time
                text(pos(1),pos(2),string(Res.time_out),"Color", colour);
            end
            
            %Mark Start Node
            start=reservation_list(1);
            pos=A.Overlay.findNodeExt(start.fromID).node.position-.5;
            text(pos(1),pos(2),string(start.time_in), "Color",[0,.7,.8]);
            
            %Mark Target Node
            final=reservation_list(end);
            pos=A.Overlay.findNodeExt(final.toID).node.position-.5;
            text(pos(1),pos(2),string(final.time_out), "Color",[1,.3,0]);
            
        end
        
        function showReservationDetails(~, reservation_list)
            
            position = [reservation_list.position]';
            path = [reservation_list.fromID;reservation_list.toID]';
            times = [reservation_list.time_in;reservation_list.time_out]';
            delayed_leaving = logical([reservation_list.delayed]');
            T = table(position,path,times,delayed_leaving);
            disp(T)
        
        end
        
        %% Debug Tools
        function findTimeIssues(A, reservation_list)
            tin= [reservation_list.time_in];
            tout=[reservation_list.time_out];
            err=[0,tout]'>[tin,Inf]';
            if any(err)
                positions_with_err=find(err);
                disp(positions_with_err)
                A.showReservationDetails(reservation_list)
                 d=r; % <-- this is a breakpoint
            end
        end
        
        function ddisp(A, msg)
            %msg=msg;
        end
        
        function cdisp(~, colour, msg)
            cprintf(colour,msg+"\n");
        end
        
        %% Update Functions
        function UpdateLocation(A, t)
            if A.idle, disp("Agent "+A.agent_id+" is IDLE at TARGET "+A.Overlay.findNodeExt(A.currentID).name); return, end
            
            %Identify the node the Agent is at at time t.
            tin=[A.reservations.time_in];
            newID = find(tin<=t,1,'last');
            
            %If t < A,reservations(1).time_in, then the agent is delayed
            if isempty(newID)% && A.reservations(1).delayed
                A.currentID = A.reservations(1).fromID;
                A.start_time = t;
            else
                %Set new location
                A.currentID = A.reservations(newID).fromID;
                A.start_time = A.reservations(newID).time_in;
            end
            
            if A.reservations(end).time_out <= t
                A.currentID = A.reservations(end).toID;
            end
            
            %{
            T=A.Overlay.findNodeExt(A.targetID);
            if T.arrival_time == t
                A.currentID = A.targetID;
                disp("Agent "+A.agent_id+" has moved to TARGET "+T.name);
                blob = 1;
            else
                for Res = A.reservations
                    if t >= Res.time_in
                        A.currentID = Res.fromID;
                        disp("Agent "+A.agent_id+" has moved to node "+A.Overlay.findNodeExt(A.currentID).name);
                        blob=1;
                        break
                    end
                end
            end
            %}
            
            %Set Agent to idle if the goal is reached
            newName = A.Overlay.findNodeExt(A.currentID).name;
            if A.currentID == A.targetID
                disp("Agent "+A.agent_id+" has moved to TARGET "+newName);
                A.idle = true;
                A.reservations = [];
                A.ROS.Coordinator.Overlay.RemovePath(A.agent_id);
            else
                disp("Agent "+A.agent_id+" has moved to node "+newName);
                A.idle = false;
            end            
        end
    end
    
end



