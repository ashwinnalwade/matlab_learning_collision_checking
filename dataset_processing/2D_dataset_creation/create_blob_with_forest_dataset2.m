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
resolution = 0.001; %map resolution
env_dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_2d_6/environments/');

%% Wall with gaps
prob_blob = 0.3;
blob_side = 0.7;

%% Forest
square_side = 0.005;
num_squares = 7; % number of rectangles

%% Create world
for i = 1:num_worlds
    i

    shape_array = [];
    % Sample a config
    if (rand() < prob_blob)
        shape_array = [shape_array get_rectangle_shape(0.5-blob_side*0.5, 0.5-blob_side*0.5, blob_side, blob_side)];
    end

    square_array = [];
    %% Pad around start to goal
    while (length(square_array) < num_squares)
        pt = [bbox(1) bbox(3)] + [bbox(2)-bbox(1) bbox(4)-bbox(3)].*rand(1,2);
        if (pt(1) > 0.5-blob_side*0.5 && pt(1) < 0.5+blob_side*0.5 && pt(2) > 0.5-blob_side*0.5 && pt(2) < 0.5+blob_side*0.5 )
            continue;
        end
        if ( norm(pt) < 0.1 || norm(pt - [1 1]) < 0.1 )
            continue;
        end
        square_array = [square_array get_rectangle_shape(pt(1)-square_side*0.5, pt(2)-square_side*0.5, square_side, square_side)];
    end
    shape_array = [shape_array square_array];
    
    map = convert_rectangle_shape_array_to_map( shape_array, bbox, resolution );

    
%     figure(1);
%     visualize_map(map);pause;
    % Filename
    filename = strcat(env_dataset, 'world_',num2str(i),'.mat');
    save(filename, 'map');
end
