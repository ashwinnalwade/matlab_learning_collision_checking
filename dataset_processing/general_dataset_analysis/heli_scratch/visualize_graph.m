%% Create regions from graphs and coll check results
% Reads a graph
clc;
clear;
close all;

%% Load graph
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_heli_terrain_3/');
set_dataset = strcat(dataset,'set_1/');

G = load_graph( strcat(set_dataset,'graph.txt') );
load(strcat(set_dataset, 'start_goal.mat'), 'start_idx', 'goal_idx');
load(strcat(set_dataset, 'edge_traj_list.mat'), 'edge_traj_list');

load( strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results' );

%% Create translators
id_list = sub2ind(size(G), [edge_traj_list.id1]', [edge_traj_list.id2]');

%% Plot paths of each world
if (1)
    world = 630;
    %for world = 1:size(coll_check_results,1)
        figure(1);
        cla;
        hold on;
        status = G;
        connected_status = coll_check_results(world,:);
        status(find(status)) = status(find(status)).*transpose(coll_check_results(world,:));
        
        for i = 1:size(edge_traj_list,1)
            if (status(edge_traj_list(i).id1, edge_traj_list(i).id2) == 0)
                %col = 'r';
                continue;
            else
                %continue;
                col = 'b';
            end
            plot3(edge_traj_list(i).traj(:,1), edge_traj_list(i).traj(:,2), -edge_traj_list(i).traj(:,4), 'Color', col, 'LineWidth', 0.25);
        end
        
        [~, path_set] = graphkshortestpaths(status, start_idx, goal_idx, 1);
        path = path_set{1};
        if(~isempty(path))
            path_edges = sub2ind(size(G), path(1:(end-1)), path(2:end));
            [~, path_edges_idx] = ismember(path_edges, id_list);
            for i = path_edges_idx
                plot3(edge_traj_list(i).traj(:,1), edge_traj_list(i).traj(:,2), -edge_traj_list(i).traj(:,4), 'g', 'LineWidth', 3);
            end
            pause;
        end
    %end
end