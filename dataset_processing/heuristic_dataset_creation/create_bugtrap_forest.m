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
env_dataset = strcat(getenv('collision_checking_dataset_folder'), '/heur_dataset_bugtrap_forest/');

%% Wall with gaps
orient = 1;
wall_width = 0.05;
back_length = 0.5; 
side_length = 0.35;

%% Forest
square_side = 0.1;
num_squares = 10; % number of rectangles
bbox_forest = [0.1 0.9 0.1 0.9];


%% Create world
for i = 1:num_worlds
    i
    shape_array = [];
    % Sample a config
    y_cnt = 0.2 + 0.6*rand();
    shape_array = [shape_array get_rectangle_bugtrap( 0.5, y_cnt, orient, wall_width, back_length, side_length)];
    shape_array = [shape_array get_square_poisson_forest( bbox_forest, square_side, num_squares )];

    map = convert_rectangle_shape_array_to_map( shape_array, bbox, resolution );

    % Filename
    filename = strcat(env_dataset, num2str(i-1),'.png');
    imwrite(flipud(map.table'), filename);
end
