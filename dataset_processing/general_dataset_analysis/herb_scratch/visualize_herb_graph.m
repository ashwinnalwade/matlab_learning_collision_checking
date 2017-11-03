%% Create regions from graphs and coll check results
% Reads a graph
clc;
clear;
close all;

%% Load graph
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_herb_3/');
set_dataset = strcat(dataset,'raw/');

G = load_graph( strcat(set_dataset,'graph.txt') );
load(strcat(set_dataset, 'graph_translator.mat'), 'vertex_id_map' ,'edge_id_map');

vertex_id_coord_set = dlmread(strcat(set_dataset,'eetransforms.txt'));

load( strcat(set_dataset, 'coll_check_results_old.mat'), 'coll_check_results' );
%% Translate coord set
coord_set = zeros(max(vertex_id_map), 3);
idx = vertex_id_map(vertex_id_coord_set(:,1));
vertex_id_coord_set(idx == 0, :) = [];
idx(idx==0) = [];
coord_set(idx, :) = vertex_id_coord_set(:,2:4);

%% Transform collision check
if (0)
    frac_flip = 0.2;
    for i = 1:size(coll_check_results, 1)
        idx_free = find(coll_check_results(i,:) == 1);
        idx_flip = idx_free(randperm(length(idx_free), round(frac_flip*length(idx_free))));
        coll_check_results(i, idx_flip) = 0;
    end
end

%% Plot 3D graph
if (0)
    scatter3(coord_set(:,1), coord_set(:,2), coord_set(:,3));
end
if (0)
    figure(1);
    hold on;
    idx = find(binornd(1, 0.2*ones(size(G,1),1)));
    view_graph_3D( G(idx,idx), coord_set(idx,:) );
    axis equal;
end
if (0)
    hold on;
    candidate_start_goal_pair_original  = ...
        [9216 9529;
        9216 6793;
        9216 6799;
        9216 271;
        2688 6793;
        2688 6799;
        2688 271];
    
    candidate_start_goal_pair = [vertex_id_map(candidate_start_goal_pair_original(:,1)) vertex_id_map(candidate_start_goal_pair_original(:,2))];
    
    scatter3(coord_set( candidate_start_goal_pair(:,1), 1), coord_set( candidate_start_goal_pair(:,1) ,2),  coord_set( candidate_start_goal_pair(:,1) ,3), 30,'r', 'filled');
    scatter3(coord_set( candidate_start_goal_pair(:,2), 1), coord_set( candidate_start_goal_pair(:,2) ,2),  coord_set( candidate_start_goal_pair(:,2) ,3), 30,'g', 'filled');
    plot3([coord_set(candidate_start_goal_pair(:,1),1)'; coord_set(candidate_start_goal_pair(:,2),1)'], [coord_set(candidate_start_goal_pair(:,1),2)'; coord_set(candidate_start_goal_pair(:,2),2)'], [coord_set(candidate_start_goal_pair(:,1),3)'; coord_set(candidate_start_goal_pair(:,2),3)'], 'color', [0.0 0.0 1.0]);
end
%% Plot paths of each world
if (0)
    idx = find(binornd(1, 0.2*ones(size(G,1),1)));
    start_idx = vertex_id_map(9216);
    goal_idx = vertex_id_map(6793);
    for i = 1:size(coll_check_results,1)
        status = G;
        status(find(status)) = status(find(status)).*transpose(coll_check_results(i,:));
        [~, path] = graphshortestpath(status, start_idx, goal_idx);
        if(~isempty(path))
            figure(1);
            cla;
            hold on;
            view_graph_3D( G(idx,idx), coord_set(idx,:) );
            axis equal;
            scatter3(coord_set( start_idx, 1), coord_set( start_idx ,2),  coord_set(start_idx,3), 30,'r', 'filled');
            scatter3(coord_set( goal_idx, 1), coord_set( goal_idx ,2),  coord_set(goal_idx,3), 30,'g', 'filled');
            plot_path_3D( path, coord_set, [0 0 1], 2 );
            pause;
        end
    end
end

%% Plot paths of each world
if (1)
    idx = find(binornd(1, 0.5*ones(size(G,1),1)));
    start_idx = vertex_id_map(9216);
    goal_idx = vertex_id_map(6793);
    for i = 1:size(coll_check_results,1)
        status = G;
        status(find(status)) = status(find(status)).*transpose(coll_check_results(i,:));
        [~, path] = graphshortestpath(status, start_idx, goal_idx);
        if(~isempty(path))
            figure(1);
            cla;
            hold on;
            view_graph_3D( status(idx,idx), coord_set(idx,:) );
            axis equal;
            scatter3(coord_set( start_idx, 1), coord_set( start_idx ,2),  coord_set(start_idx,3), 30,'r', 'filled');
            scatter3(coord_set( goal_idx, 1), coord_set( goal_idx ,2),  coord_set(goal_idx,3), 30,'g', 'filled');
            plot_path_3D( path, coord_set, [0 0 1], 2 );
            pause;
        end
    end
end

%% Plot collision edges of each world
if (0)
    idx = find(binornd(1, 0.3*ones(size(G,1),1)));
    start_idx = vertex_id_map(9216);
    goal_idx = vertex_id_map(6793);
    for i = 1:size(coll_check_results,1)
        status = G;
        status(find(status)) = status(find(status)).*transpose(~coll_check_results(i,:));
        figure(1);
        cla;
        hold on;
        view_graph_3D( status(idx,idx), coord_set(idx,:) );
        axis equal;
        scatter3(coord_set( start_idx, 1), coord_set( start_idx ,2),  coord_set(start_idx,3), 30,'r', 'filled');
        scatter3(coord_set( goal_idx, 1), coord_set( goal_idx ,2),  coord_set(goal_idx,3), 30,'g', 'filled');
        pause;
    end
end
