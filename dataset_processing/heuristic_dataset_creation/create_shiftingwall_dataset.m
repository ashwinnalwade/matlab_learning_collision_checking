%% Create a wall with gaps in conjunction with poisson forest
% Create a wall with gaps and induce stochasticity by randomly sampling
% from a list of configurations. A small list enforces a natural structural
% corrleation. The poisson forest is to additionally increase difficulty of
% the problem
clc;
clear;
close all;

%% World
rng(1);
bbox = [0 1 0 1]; %unit bounding box
num_worlds = 1000;
resolution = 0.005; %map resolution
env_dataset = strcat(getenv('collision_checking_dataset_folder'), '/heur_dataset_shifting_wall/');

%% Wall with gaps
width = 0.2;
gap_height = 0.1;


%% Create world
for i = 1:num_worlds
    i
    shape_array = [];
    % Sample a config
    shape_array = [shape_array get_wall_with_shifting_gaps( bbox, 0.5, width, gap_height, [0.1 0.9] )];

    map = convert_rectangle_shape_array_to_map( shape_array, bbox, resolution );

    % Filename
    filename = strcat(env_dataset, num2str(i-1),'.png');
    imwrite(flipud(map.table'), filename);
end
