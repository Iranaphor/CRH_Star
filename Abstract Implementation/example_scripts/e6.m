% -- Scalability --
addpath(genpath('./YAMLMatlab_0.4.3'));
Config = ReadYaml('meta.yaml');

Config.meta.total_cycles = 50;
Config.meta.total_row_monitoring = 0;
Config.meta.total_crop_monitoring = 0;

mkdir('results/e6/')
folder="results/e6/";

%-----------------------------------------------------
Config.meta.pointset = 'riseholme_poly_act_sim.tmap';

Config.meta.total_agents = 1; Config.meta.total_logistics = 1;
for i=1:5; [~,TR,~,~,~]=roscore(Config); TRL(i)=mean(TR); end
save(folder+"B-1.mat", 'TRL');

Config.meta.total_agents = 5; Config.meta.total_logistics = 5;
for i=1:5; [~,TR,~,~,~]=roscore(Config); TRL(i)=mean(TR); end
save(folder+"B-2.mat", 'TRL');


Config.meta.total_agents = 10; Config.meta.total_logistics = 10;
for i=1:5; [~,TR,~,~,~]=roscore(Config); TRL(i)=mean(TR); end
save(folder+"B-3.mat", 'TRL');


Config.meta.total_agents = 25; Config.meta.total_logistics = 25;
for i=1:5; [~,TR,~,~,~]=roscore(Config); TRL(i)=mean(TR); end
save(folder+"B-4.mat", 'TRL');

Config.meta.total_agents = 50; Config.meta.total_logistics = 50;
for i=1:5; [~,TR,~,~,~]=roscore(Config); TRL(i)=mean(TR); end
save(folder+"B-5.mat", 'TRL');

Config.meta.total_agents = 100; Config.meta.total_logistics = 100;
for i=1:5; [~,TR,~,~,~]=roscore(Config); TRL(i)=mean(TR); end
save(folder+"B-6.mat", 'TRL');

