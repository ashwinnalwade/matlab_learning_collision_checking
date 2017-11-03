clc;
clear;
close all;

%% Load
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_wire_1/');
load(strcat(dataset,'real_data/explicit_graph.mat'), 'G', 'edge_traj_list', 'start_idx', 'goal_idx');
real_results = dlmread(strcat(dataset,'real_data/17.txt'));
real_results2 = dlmread(strcat(dataset,'real_data/14.txt'));

set_dataset = strcat(dataset,'set_1/');

%% Save stuff
save_graph( strcat(set_dataset, 'graph.txt'), G );
save(strcat(set_dataset, 'start_goal.mat'), 'start_idx', 'goal_idx');
save(strcat(set_dataset, 'edge_traj_list.mat'), 'edge_traj_list');

%% Priors
alpha1 = 0.0;
alpha2 = 0.0;
test_bias = alpha1*real_results + alpha2*real_results2 + (1-alpha1-alpha2)*0.5*ones(size(real_results));
num_worlds = 1000;

%% Create dataset
id_list = sub2ind(size(G), [edge_traj_list.id1]', [edge_traj_list.id2]');
edges = find(G);
coll_check_results = [];
while(size(coll_check_results,1) < num_worlds)
    size(coll_check_results,1)
    if (size(coll_check_results,1) == 0)
        connected_status = logical(real_results);
    else
        connected_status = logical(binornd(1, test_bias));
    end
    
    status = G;
    status(id_list(~connected_status)) = 0;
    
    [~, path] = graphshortestpath(status, start_idx, goal_idx);
    if (~isempty(path))
        coll_check_results = [coll_check_results; transpose(full(status(edges)))];
    end
end
coll_check_results = logical(coll_check_results);

%% Save results
save(strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results');