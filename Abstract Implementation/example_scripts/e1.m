%-- Base vs Continuous Assignment --
addpath(genpath('./YAMLMatlab_0.4.3'));
Config = ReadYaml('meta.yaml');

Config.meta.total_row_monitoring = 0;
Config.meta.total_crop_monitoring = 0;

Config.meta.rng_seed = -1;

mkdir('results/e1/')
Config.meta.pointset = 'riseholme_poly_act_sim.tmap';

uca=[0,1,0,1,0,1];
score=[5,5,10,10,20,20];
x=0;
for i = 1:6
    for j=1:10
        x=x+1;
        Config.heuristics.use_continuous_assignment = uca(i);
        Config.meta.total_cycles = 20;
        Config.meta.total_agents = score(i);
        Config.meta.total_logistics = score(i);
        [~,~,~,~,~,T]=roscore(Config);
        save("results/e1/"+x+"-"+i+".mat", 'T');
    end
end