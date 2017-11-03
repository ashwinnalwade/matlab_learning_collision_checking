%% Create regions from graphs and coll check results
% Reads a graph
clc;
clear;
close all;

%% Load graph
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_heli_terrain_4/');
set_dataset = strcat(dataset,'set_1/');

G = load_graph( strcat(set_dataset,'graph.txt') );
load(strcat(set_dataset, 'start_goal.mat'), 'start_idx', 'goal_idx');
load(strcat(set_dataset, 'edge_traj_list.mat'), 'edge_traj_list');

load( strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results' );

%% Create translators
id_list = sub2ind(size(G), [edge_traj_list.id1]', [edge_traj_list.id2]');

%% Plot paths of each world
num_solved = 0;
unsolved_worlds = [];
if (1)
    for world = 1:size(coll_check_results,1)
        world
        status = G;
        connected_status = coll_check_results(world,:);
        status(find(status)) = status(find(status)).*transpose(coll_check_results(world,:));
        
        [~, path] = graphshortestpath(status, start_idx, goal_idx);
        %[~, path_set] = graphkshortestpaths(status, start_idx, goal_idx, 1);
        if(~isempty(path))
            num_solved = num_solved + 1;
        else
            unsolved_worlds = [unsolved_worlds;  world];
        end
    end
end