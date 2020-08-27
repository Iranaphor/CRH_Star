% Base vs Context-Dependent Heuristics
addpath(genpath('./YAMLMatlab_0.4.3'));
Config = ReadYaml('meta.yaml');

Config.meta.total_agents = 10;
Config.meta.total_logistics = 4;
Config.meta.total_row_monitoring = 3;
Config.meta.total_crop_monitoring = 3;

Config.meta.rng_seed = -1;

Config.meta.pointset = 'riseholme_poly_act_sim.tmap';
mkdir('results/e3/')
folder="results/e3"; maxI=40;

%------------------------------------------
Config.meta.total_cycles = 50;

Config.heuristics.dependent_heuristic = 0;
for i=1:maxI; [TD,~,~,~,~]=roscore(Config); TotalDelay(i)=mean(TD); end
save(folder+"/1.mat", 'TotalDelay');

Config.heuristics.dependent_heuristic = 1;
for i=1:maxI; [TD,~,~,~,~]=roscore(Config); TotalDelay(i)=mean(TD); end
save(folder+"/2.mat", 'TotalDelay');

%------------------------------------------
Config.meta.total_cycles = 100;

Config.heuristics.dependent_heuristic = 0;
for i=1:maxI; [TD,~,~,~,~]=roscore(Config); TotalDelay(i)=mean(TD); end
save(folder+"/3.mat", 'TotalDelay');

Config.heuristics.dependent_heuristic = 1;
for i=1:maxI; [TD,~,~,~,~]=roscore(Config); TotalDelay(i)=mean(TD); end
save(folder+"/4.mat", 'TotalDelay');

%------------------------------------------
Config.meta.total_cycles = 200;

Config.heuristics.dependent_heuristic = 0;
for i=1:maxI; [TD,~,~,~,~]=roscore(Config); TotalDelay(i)=mean(TD); end
save(folder+"/5.mat", 'TotalDelay');

Config.heuristics.dependent_heuristic = 1;
for i=1:maxI; [TD,~,~,~,~]=roscore(Config); TotalDelay(i)=mean(TD); end
save(folder+"/6.mat", 'TotalDelay');

%------------------------------------------

