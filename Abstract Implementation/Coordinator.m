classdef Coordinator < handle
    %COORDINATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Network
        Overlay
        ROS
        OverlayUUID = 0;
    end
    
    methods
        function C = Coordinator(ROS), C.ROS=ROS; end
        
        
        function Subscribe_ReservationsUpdate(C, reservation_list)
            aid=reservation_list(1).agent_id;
            
            disp("Agent "+aid+" reserved "+length(reservation_list)+" edges.")
            
            notifications = {};
            agents = [];
            
            %Remove existing reservations by the agent publishing the path
            C.Overlay.RemovePath(aid);
            
            %For each edge taken by reservation
            for i = 1:length(reservation_list)
                r = reservation_list(i);
                
                %Manage Extra Delay Reservation
                if r.fromID == r.toID
                    continue
                end
                
                %Identify edge and conflicts
                EExt = C.Overlay.findEdgeExt(r.fromID, r.toID);
                existingConflicts = EExt.IsEmpty(r.time_in, r.time_out);
                
                %Add conflict details to list
                if ~isempty(existingConflicts)
                    for oldR = existingConflicts
                        EExt.total_conflicts = EExt.total_conflicts + 1;
                        notifications{end+1} = oldR;
                        agents(end+1) = oldR.agent_id;
                    end
                end
                
                %Add the new reservation
                EExt.AddReservation(r.agent_id, r.time_in, r.time_out, r.CRH, r.position, r.uuid);
                
            end
            
            %Get list of unique agents to notify
            agents = unique(agents);
            if ~isempty(agents)
                disp("Agent "+aid+" caused "+length(notifications)+...
                    " conflicts with agents: " + ...
                    "["+strjoin(string(agents), ',')+"]")
            else
                disp("Agent "+aid+" caused no conflicts.")
            end
            
            
            %Update overlay UUID to show change in reservations. (unused)
            C.OverlayUUID = C.OverlayUUID + 1;
            
            %For each unique agent which has lost a reservation
            for a = agents
                
                %Identify failed reservations to send to agent
                agent_failed_reservations = {};
                for n = notifications
                    if n{1}.agent_id == a
                        if C.existsConflict(n{1}, aid)
                            agent_failed_reservations{end+1} = n{1};
                        end
                    end
                end
                
                %Notify agent of its failed reservations
                if ~isempty(agent_failed_reservations)
                    C.publishNotifications(a, agent_failed_reservations);
                end
%{
%                 %Identify if each notification is still required
%                 for i = 1:length(notifications)
%                     n=notifications(i);
%                     EExt = C.Overlay.findEdgeExt(n{1}.fromID, n{1}.toID);
%                     existingConflicts = EExt.IsEmpty(n{1}.time_in, n{1}.time_out);
%                     
%                     %eR is empty -< replan not needed
%                     if isempty(existingConflicts), notifications{i}=[]; end
%                     
%                     %eR is has only aid -< replan not needed
%                     if length(existingConflicts)==1
%                         if existingConflicts(1).agent_id == aid
%                             notifications{i}=[];
%                         end
%                     end
%                     
%                     %eR is has aid + conflict -< replan needed
%                     %continue
%                     
%                 end
%                 
%                 %Remove empty notifications
%                 for i = flip(1:length(notifications))
%                     if isempty(notifications{i})
%                         notifications(i)=[];
%                     end
%                 end
%}
            end
            
        end
        
        function conflictExists = existsConflict(C, n, aid)
            EExt = C.Overlay.findEdgeExt(n.fromID, n.toID);
            existingConflicts = EExt.IsEmpty(n.time_in, n.time_out);
            
            % < 2 reservation -< conflict impossible -< replan not needed
            conflictExists = length(existingConflicts) >= 2;
            
            if conflictExists
                ec=[existingConflicts.agent_id];
                conflictExists = all([any(ec==n.agent_id),any(ec==aid)]);
            end
            
            %{
            The point is to only allow reservations to be used for
            replanning if the reservation is still valid
            
            it is still valid if the edge has the reservation and the
            request in place
            
            so existingConflicts must contain a reservation with the id of
            the one requesting the edge, and the id of the agent who is
            being overtaken
            
            must it also have no other reservations present? it will not in
            the end
            %}
            
            if length(existingConflicts) > 2
                disp("This is probably an issue... shouldnt exceed 2 reservations")
            end
                    
                    
        end
        
        
        function publishNotifications(C, agent_id, reservations)
            
            C.ROS.Topic_publishFailedReservations(agent_id, reservations);
            
        end
    end
end



















