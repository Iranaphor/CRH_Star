% Base vs Dynamic Scoring
addpath(genpath('./YAMLMatlab_0.4.3'));
Config = ReadYaml('meta.yaml');

Config.meta.total_cycles = 50;
Config.meta.total_agents = 10;
Config.meta.total_logistics = 10;
Config.meta.total_row_monitoring = 0;
Config.meta.total_crop_monitoring = 0;

Config.meta.rng_seed = -1;

mkdir('results/e2/')
Config.meta.pointset = 'riseholme_poly_act_sim.tmap';

uca=[0,1,0,1,0,1];
heur=[	{'euclidian'},{'euclidian'},...
		{'optimal'},{'optimal'},...
		{'planning'},{'planning'}];

TotalDelay=[];
x=0;
for i = 1:6
    for run=1:5
        x=x+1;
        Config.heuristics.use_continuous_assignment = uca(i);
        Config.heuristics.independent_heuristic = heur(i);
        [TD,~,~,~,~,~]=roscore(Config);
        TotalDelay(run)=mean(TD);
    end
    save("results/e2/"+i+".mat", 'TotalDelay');
end