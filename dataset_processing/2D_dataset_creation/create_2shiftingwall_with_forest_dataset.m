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
env_dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_2d_9/');

%% Wall with gaps
width = 0.15;
gap_height = 0.1;

%% Forest
square_side = 0.05;
num_squares = 3; % number of rectangles
bbox_forest1 = [0.0 0.25 0.1 0.9];
bbox_forest2 = [0.35 0.65 0.1 0.9];
bbox_forest3 = [0.75 1.0 0.1 0.9];

%% Create world
for i = 1:num_worlds
    i
    shape_array = [];
    % Sample a config
    shape_array = [shape_array get_wall_with_shifting_gaps( bbox, 0.3, width, gap_height, [0.7 0.9] )];
    shape_array = [shape_array get_wall_with_shifting_gaps( bbox, 0.7, width, gap_height, [0.1 0.3] )];
    shape_array = [shape_array get_square_poisson_forest( bbox_forest1, square_side, num_squares )];
    shape_array = [shape_array get_square_poisson_forest( bbox_forest2, square_side, num_squares )];
    shape_array = [shape_array get_square_poisson_forest( bbox_forest3, square_side, num_squares )];

    map = convert_rectangle_shape_array_to_map( shape_array, bbox, resolution );

    % Filename
    filename = strcat(env_dataset, num2str(i-1),'.png');
    imwrite(flipud(map.table'), filename);
end
