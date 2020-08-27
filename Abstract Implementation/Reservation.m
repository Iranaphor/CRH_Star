classdef Reservation < handle
    
    properties
        fromID
        toID
        time_in
        time_out
        agent_id
        CRH
        position
        delayed = 0
        
        %verification data
        uuid
    end
    
    methods
        function R = Reservation(node1, node2, time_in, time_out, agent_id, CRH, position)
            R.fromID = node1;
            R.toID = node2;
            R.time_in = time_in;
            R.time_out = time_out;
            R.agent_id = agent_id;
            R.CRH = CRH;
            R.position = position;
        end
    end
end

