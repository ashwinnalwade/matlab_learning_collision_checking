clc;
clear;
close all;

%% Load
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_heli_terrain_4/');
load(strcat(dataset,'original_dataset/explicit_graph.mat'), 'G', 'edge_traj_list', 'start_idx', 'goal_idx');
set_dataset = strcat(dataset,'set_1/');

%% Save stuff
save_graph( strcat(set_dataset, 'graph.txt'), G );
save(strcat(set_dataset, 'start_goal.mat'), 'start_idx', 'goal_idx');
save(strcat(set_dataset, 'edge_traj_list.mat'), 'edge_traj_list');

%% Create translators
id_list = sub2ind(size(G), [edge_traj_list.id1]', [edge_traj_list.id2]');

%% Read and save results
num_env = 1000;
edges = find(G);
coll_check_results = false(num_env, length(edges));
for i = 1:num_env
    i
    result = dlmread(strcat(dataset,'original_dataset/results/',num2str(i-1),'.txt'));
    connected_status = logical(result);
    status = G;
    status(id_list(~connected_status)) = 0;
    coll_check_results(i, :) = transpose(full(status(edges)));
end
save(strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results');