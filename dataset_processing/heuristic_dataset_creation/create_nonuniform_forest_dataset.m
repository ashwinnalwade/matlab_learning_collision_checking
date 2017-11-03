%% Create a nonuniform forest
% Create a non-uniform forest of square obstacles by defining a mixture of
% gaussians. A poisson dist is added to increase difficulty of
% the problem
clc;
clear;
close all;

%% World
rng(1);
bbox = [0 1 0 1]; %unit bounding box
num_worlds = 1000;
resolution = 0.005; %map resolution
env_dataset = strcat(getenv('collision_checking_dataset_folder'), '/heur_dataset_nonuniform_forest/');

%% Wall with gaps
mu = [0.3 0.3; 
      0.1 0.5;
      0.9 0.5;
      0.7 0.7];
sigma = repmat(0.004*eye(2), [1 1 size(mu,1)]);
p = ones(1,size(mu,1));
obj = gmdistribution(mu,sigma,p);

square_side = 0.1;
num_squares = 15; % number of rectangles
num_squares_poisson = 1;
bbox_forest = [bbox(1) + square_side*0.75 bbox(2) - square_side*0.75 bbox(3) + square_side*0.75 bbox(4) + square_side*0.75];

%% Create world
for i = 1:num_worlds
    i
    
    shape_array = [];
    % Sample a config
    shape_array = [shape_array get_square_nonuniform_forest( bbox_forest, square_side, num_squares, obj );];
    shape_array = [shape_array get_square_poisson_forest( bbox_forest, square_side, num_squares_poisson )];
    shape_array = [shape_array get_rectangle_shape(0.65, 0.25, 0.35, 0.05)];
    shape_array = [shape_array get_rectangle_shape(0.0, 0.75, 0.35, 0.05)];

    map = convert_rectangle_shape_array_to_map( shape_array, bbox, resolution );
%     figure(1);
%     visualize_map(map); 
%     pause();

    % Filename
    filename = strcat(env_dataset, num2str(i-1),'.png');
    imwrite(flipud(map.table'), filename);
end


