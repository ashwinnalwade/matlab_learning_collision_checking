clc;
clear;
close all;

%% World
rng(1);
num_worlds = 5000;
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_2d_8/');
set_dataset = strcat(dataset,'set_1/');
env_dataset = strcat(dataset,'environments/');

%% Create graph
bbox = [0 1 0 1];
N = 150;
eta = 1;
[ G, coord_set ] = rgg( bbox, N, eta);

%% Start and goal
[~, start_idx] = min(pdist2([0 0], coord_set), [], 2);
[~, goal_idx] = min(pdist2([1 1], coord_set), [], 2);
% G(start_idx, goal_idx) = norm(coord_set(start_idx, :) - coord_set(goal_idx, :));
% G(goal_idx, start_idx) = G(start_idx, goal_idx);

save_graph( strcat(set_dataset, 'graph.txt'), G );
save(strcat(set_dataset, 'coord_set.mat'), 'coord_set');
save(strcat(set_dataset, 'start_goal.mat'), 'start_idx', 'goal_idx');

%% Create world
edges = find(G);
coll_check_results = zeros(num_worlds, length(edges));

map_set = [];
for i = 1:num_worlds
    % Filename
    filename = strcat(env_dataset, 'world_',num2str(i),'.mat');
    load(filename, 'map');
    map_set{i} = map;
end

parfor i = 1:num_worlds
    i
    % Filename
    filename = strcat(env_dataset, 'world_',num2str(i),'.mat');
    map = map_set{i};
    % Collision check
    line_coll_check_fn = @(parent, child) check_coll_line_map( parent, child, map );
    pt_coll_check_fn = @(pt) check_coll_wpset_map( pt, map ); 
    status = collision_check_2d_graph( G, coord_set, line_coll_check_fn, pt_coll_check_fn );
    
    coll_check_results(i, :) = transpose(full(status(edges)));
end
%dlmwrite( strcat(set_dataset, 'coll_check_results.txt'), coll_check_results );
coll_check_results = logical(coll_check_results);
save(strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results');