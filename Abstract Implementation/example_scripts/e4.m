% Base vs Combinatorial Heuristics
addpath(genpath('./YAMLMatlab_0.4.3'));
Config = ReadYaml('meta.yaml');

Config.meta.total_cycles = 20;
Config.meta.total_agents = 10;
Config.meta.total_logistics = 10;
Config.meta.total_row_monitoring = 0;
Config.meta.total_crop_monitoring = 0;

Config.meta.rng_seed = -1;

mkdir('results/e4/')
folder="results/e4/";
Config.meta.pointset = 'riseholme_poly_act_sim.tmap';
runsCount=10;
% -----------------------------------------------------

Config.heuristics.independent_heuristic = {'euclidian_distance'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL1(i)=mean(TR); end

Config.heuristics.independent_heuristic = {'optimal_route_length'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL2(i)=mean(TR); end

Config.heuristics.independent_heuristic = {'planning_time'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL3(i)=mean(TR); end
save(folder+"1.mat", 'TRL1', 'TRL2', 'TRL3');

Config.heuristics.independent_heuristic = {'euclidian_distance','optimal_route_length','planning_time'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL(i)=mean(TR); end
save(folder+"2.mat", 'TRL');

% -----------------------------------------------------

Config.heuristics.independent_heuristic = {'agent_id'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL1(i)=mean(TR); end

Config.heuristics.independent_heuristic = {'optimal_route_length'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL2(i)=mean(TR); end
save(folder+"3.mat", 'TRL1', 'TRL2');

Config.heuristics.independent_heuristic = {'agent_id','optimal_route_length'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL(i)=mean(TR); end
save(folder+"4.mat", 'TRL');

% -----------------------------------------------------

Config.heuristics.independent_heuristic = {'agent_id'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL1(i)=mean(TR); end

Config.heuristics.independent_heuristic = {'euclidian_distance'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL2(i)=mean(TR); end
save(folder+"5.mat", 'TRL1', 'TRL2');

Config.heuristics.independent_heuristic = {'agent_id', 'euclidian_distance'};
for i=1:runsCount; [~,TR,~,~,~]=roscore(Config); TRL(i)=mean(TR); end
save(folder+"6.mat", 'TRL');
