classdef EdgeExt < handle
    
    properties
        reservations = [] %[agent, start_time, end_time]
        edge
        time_weight
        total_reservations = 0;
        total_conflicts = 0;
        deadlock_overcome = false;
    end
    
    methods
        function EE = EdgeExt(edge)
            EE.edge = edge;
            EE.time_weight = floor(edge.weight * 10); %this can be changed later
        end
        
        function AddReservation(EE, agent, start_time, end_time, CRH, position, uuid)
			EE.reservations = [EE.reservations; [agent, start_time, end_time, CRH, position, uuid]];
            EE.total_reservations = EE.total_reservations + 1;
        end
        
        function Purge(EE)
			EE.reservations = [];
            EE.total_reservations = 0;
            EE.total_conflicts = 0;
        end
        
        function PurgeAgent(EE, agent_id)
            if ~isempty(EE.reservations)
                EE.reservations(EE.reservations(:,1) == agent_id,:) = [];
            end
        end
        
        function PurgeOld(EE, t)
            if ~isempty(EE.reservations)
                EE.reservations(EE.reservations(:,3) <= t,:) = [];
            end
        end
        
        function result = IsEmpty(EE, fromT, tillT)
            result = IsEmpty2(EE, fromT, tillT, EE.edge.node1.id);
        end
        
        function result = IsEmpty2(EE, fromT, tillT, fromID)
             %{
          res.timefrom[======================]res.timetill
                   |----------------------------|
                |-----------|          |----------|
            |-----|        |------------|        |------|
                 tillT                        fromT
            
            if ~(tillT < res.timefrom | fromT > res.timetill)
                bad
            end
            %}
            res = EE.reservations;
            if isempty(res), result = []; return, end
            
            BEFORE = tillT <= res(:,2);
            AFTER = fromT >= res(:,3);
            conflicts = res(~( BEFORE | AFTER),:);
            
            edgeNames = [EE.edge.node1.id, EE.edge.node2.id];
            
            for i = 1:size(conflicts,1)
                r=num2cell([fromID, edgeNames(edgeNames~=fromID),...
                    conflicts(i,[2,3,1,4,5])]);
                result(i) = Reservation(r{:});
                result(i).uuid = conflicts(i,6);
            end
            
            if ~exist('result','var'), result = []; return, end
        end
        
        function newFromT = FindAvailablePeriod(EE, fromT, tillT)
            if isempty(EE.reservations), newFromT=fromT; return; end
            
            %Sort reservations
            R = EE.reservations;
            [~,g]=sort(R(:,3));
            R=R(g,:);
            
            %Remove any reservations before the current request
            R(R(:,3) <= fromT,:) = []; %R(R(:,3) < fromT,:) = [];
            
            %If no more reservations, return (this should never be hit)
            if isempty(R), newFromT=fromT; return; end
            
            %Identify space required
            duration=tillT-fromT;
            
            %Identify spaces available
            spaces=[R(2:end,2);inf]-R(:,3);
            
            %Find next space available greater than the space required
            if spaces(find(spaces>duration,1)) == Inf
                %If no space between reservations, take exit from last
                newFromT = R(end,3)+1;
            else
                newFromT = R(find(spaces>duration,1),3)+1;
            end
            
        end
        
        function bool = IsEqual(EE, e)
            
            EEnames = [EE.edge.node1.id, EE.edge.node2.id];
            enames = [e.node1.id, e.node2.id];

            bool = any([all(EEnames == enames), all(EEnames == flip(enames))]);
            
        end
        
    end
end
