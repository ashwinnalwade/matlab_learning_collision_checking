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
env_dataset = strcat(getenv('collision_checking_dataset_folder'), '/heur_dataset_3bugtrap/');

%% Create world
for i = 1:num_worlds
    i
    shape_array = [];

    x_cnt = 0.5;
    y_cnt = 0.2 + 0.6*rand();
    shape_array = [shape_array get_rectangle_bugtrap( x_cnt, y_cnt, 1, 0.025, 0.5, 0.25)];

    x_cnt = 0.25*rand();
    y_cnt = 0.35 + 0.35*rand();
    shape_array = [shape_array get_rectangle_bugtrap( x_cnt, y_cnt, 2, 0.025, 0.3, 0.35)];
    
    x_cnt = 0.75 + 0.25*rand();
    y_cnt = 0.35 + 0.35*rand();
    shape_array = [shape_array get_rectangle_bugtrap( x_cnt, y_cnt, 2, 0.025, 0.3, 0.35)];
    
    map = convert_rectangle_shape_array_to_map( shape_array, bbox, resolution );

%     figure(1);
%     visualize_map(map); 
%     pause();

    % Filename
    filename = strcat(env_dataset, num2str(i-1),'.png');
    imwrite(flipud(map.table'), filename);
end
