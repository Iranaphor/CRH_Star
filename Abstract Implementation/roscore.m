function [TotalDelay, TotalReplans, AgentID, Deadlocks, Times, T] = roscore(varargin)
    clc; 
    close all; 

    %% Load Configuration File
    varargin=[];
    if ~isempty(varargin)
        Config = varargin{1};
    else
        addpath(genpath('./YAMLMatlab_0.4.3'));
        Config = ReadYaml('configuration_files/meta.yaml');
    end
    Config.meta.total_cycles = 5000;
    Config.meta.pointset = 'riseholme_poly_act_sim.tmap';
    
    for i=1:length(Config.heuristics.independent_heuristic)
        Config.heuristics.independent_heuristic{i} = ...
            string(Config.heuristics.independent_heuristic{i});
    end
    Config.heuristics.independent_heuristic=...
        [Config.heuristics.independent_heuristic{:}];
    
    %% Define random seed
    if Config.meta.rng_seed ~= -1
        rng(Config.meta.rng_seed);
    else
       rng('shuffle');
    end

    %% Load Map File
    disp("Loading Pointset: "+string(Config.meta.pointset(1:end-5)));
    try %#ok<TRYNC>
        pointset = eval("Config.meta."+string(Config.meta.pointset(1:end-5)));
        Config.meta.storageNodes   = pointset.storageNodes;
        Config.meta.rowNodes_start = pointset.rowNodes_start;
        Config.meta.rowNodes_end   = pointset.rowNodes_end;
        Config.meta.cropNodes  = pointset.cropNodes;
        Config.meta.bad_order  = pointset.bad_order;
    end
    Meta = Config.meta;
    Heuristics = Config.heuristics;
    
    [NodeList, NodeNameList, AdjacencyMatrix] = ReadTmap(Meta.pointset);


    %% Launch ROS Core
    ROSCore = ROSSim(NodeList, NodeNameList, AdjacencyMatrix);

    %Display map based on Config file demands
    if Meta.show_map_on_startup
        if Meta.show_node_names, ROSCore.Show_Map;
        else, ROSCore.Coordinator.Network.quickPlot; end
    end
    clear NodeList NodeNameList AdjacencyMatrix


    %% Launch Agents
    A{Meta.total_agents} = 0;
    Type_Count=[Meta.total_logistics, Meta.total_crop_monitoring, Meta.total_row_monitoring];
    Type_Label=["logistics","crop_monitoring","row_monitoring"];
    Types = repelem(Type_Label,Type_Count);
    for i = 1:length(Types)
        ROSCore.Launch_TaskAgent(ROSCore.getRandomNode, Heuristics, Types(i))
        A{(Meta.total_agents-i)+1} = ROSCore.Agent_List(end);
    end
    
    %% Set initial navigation target for each Agent
    for i = 1:length(ROSCore.Agent_List)    
        A{i}.NavigateTo_static(ROSCore.getMeaningfulNode(A{i}, Config), 0);
        Path_Log{i}{1} = A{i}.reservations; 
    end
    clear aid

    %% Begin Simulation
    T = 0;
    AFin = zeros(1,length(A));
    Idle = false(1,length(A));
    Planned = true(1,length(A));
    [TotalReplans, TotalDelay, AgentID, Deadlocks, Times] = ...
        deal(zeros(1,Meta.total_cycles));
    
    for cycle = 1:Meta.total_cycles
        t=tic;

        %Find next timestep
        for i = 1:length(ROSCore.Agent_List)
            AFin(i) = A{i}.Overlay.findNodeExt(A{i}.targetID).arrival_time;
            try
                Idle(i) = A{i}.idle;
            catch
                disp("Hi")
            end
            if ~isempty(A{i}.reservations)
                Path_Log{i}{end+1} = A{i}.reservations;
            end
        end

        %Move to next timestep with event
        if Heuristics.use_continuous_assignment
            
            [~,m]=min(AFin);
            T = ROSCore.TimeStep(AFin(m)-T);
            
        else
            
            if all(Planned)
                T = ROSCore.TimeStep((max(AFin)-T));
                disp("NEW BATCH STARTED")
                Planned = false(1,length(A));
                cycle = cycle - 1;
                continue
            else
                T = ROSCore.TimeStep(0);
                [~,m]=min(AFin);
                Planned(m) = true;
            end
            
        end
        
        %Save details
        TotalReplans(cycle) = A{m}.total_replans;
        TotalDelay(cycle) = (T - A{m}.optimal_arrival_time);
        AgentID(cycle) = m;
        Deadlocks(cycle) = A{m}.total_deadlocks_overcome;
        Times(cycle) = min(AFin)-A{m}.start_time;
        
        %Navigate to new random target
        if T >= A{m}.Overlay.findNodeExt(A{m}.targetID).arrival_time
            A{m}.NavigateTo_static(ROSCore.getMeaningfulNode(A{m}, Config), T);
        end
        
        disp("Time taken to define new goal: "+toc(t))
    end
    
    %Show Route Details
    %ROSCore.Show_PredMovement(Path_Log, 0, 10, T)

	%Show Reservation Details
    ROSCore.Analyse_ReservationActivity
    %ROSCore.Analyse_ConflictActivity

end