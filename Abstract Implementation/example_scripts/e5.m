% Deadlocks Managed
addpath(genpath('./YAMLMatlab_0.4.3'));
Config = ReadYaml('meta.yaml');

Config.meta.total_cycles = 100;

Config.meta.total_agents = 10;
Config.meta.total_logistics = 10;
Config.meta.total_row_monitoring = 0;
Config.meta.total_crop_monitoring = 0;
Config.meta.total_crop_monitoring = 0;

Config.meta.rng_seed = -1;

mkdir('results/e5/')
for i = 1:50
    [~,~,~,deadlocks,~]=roscore(Config);
    save("results/e5/"+i+".mat", 'deadlocks');
end