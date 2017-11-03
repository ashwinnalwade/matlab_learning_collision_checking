%% Create regions from graphs and coll check results
% Reads a graph
clc;
clear;
close all;

%% Load graph
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_xyh_1/');
set_dataset = strcat(dataset,'set_1/');
env_dataset = strcat(dataset,'environments/');

G = load_graph( strcat(set_dataset,'graph.txt') );
load(strcat(set_dataset, 'start_goal.mat'), 'start_idx', 'goal_idx');
load(strcat(set_dataset, 'edge_traj_list.mat'), 'edge_traj_list');

load( strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results' );

%% Create translators
id_list = sub2ind(size(G), [edge_traj_list.id1]', [edge_traj_list.id2]');

%% Plot paths of each world
if (1)
    for i = 1:size(coll_check_results,1)
        status = G;
        status(find(status)) = status(find(status)).*transpose(coll_check_results(i,:));
        [~, path] = graphshortestpath(status, start_idx, goal_idx);
        if(~isempty(path))
            % Filename
            filename = strcat(env_dataset, 'world_',num2str(i),'.mat');
            load(filename, 'map');
            figure(1);
            cla;
            hold on;
            visualize_map(map);
            
            path_edges = sub2ind(size(G), path(1:(end-1)), path(2:end));
            [~, path_edges_idx] = ismember(path_edges, id_list);
            for i = path_edges_idx
                plot(edge_traj_list(i).traj(:,1), edge_traj_list(i).traj(:,2), 'g');
            end
            
            pause;
        end
    end
end