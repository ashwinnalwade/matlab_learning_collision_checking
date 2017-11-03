clc;
clear;
close all;

%% World
rng(1);
num_worlds = 1000;
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_xyh_2/');
set_dataset = strcat(dataset,'set_1/');
env_dataset = strcat(dataset,'environments/');

average_map = [];
for i = 1:7
    % Filename
    filename = strcat(env_dataset, 'world_',num2str(i),'.mat');
    load(filename, 'map');
    if (isempty(average_map))
        average_map = map;
    else
        average_map.table = average_map.table + map.table;
    end
end

average_map.table = average_map.table / num_worlds;
figure(1);
hold on;
visualize_map(average_map);

%% Plot lattice
G = load_graph( strcat(set_dataset,'graph.txt') );
load(strcat(set_dataset, 'start_goal.mat'), 'start_idx', 'goal_idx');
load(strcat(set_dataset, 'edge_traj_list.mat'), 'edge_traj_list');
id_list = sub2ind(size(G), [edge_traj_list.id1]', [edge_traj_list.id2]');


plot_edge_traj_list( edge_traj_list, [0.6 0.6 0.8] );
coord_set = [];
for i = 1:size(edge_traj_list,1)
    coord_set = [coord_set;
        edge_traj_list(i).traj(1,1) edge_traj_list(i).traj(1,2);
        edge_traj_list(i).traj(end,1) edge_traj_list(i).traj(end,2)];
end
scatter(coord_set(:,1), coord_set(:,2),30,'k', 'filled');
axis off;

%% Plot regions
load(strcat(set_dataset, 'path_library.mat'), 'path_library');
cmap = parula(50);
for i = 1:50
    path = path_library{i};
    path_edges = sub2ind(size(G), path(1:(end-1)), path(2:end));
    [~, path_edges_idx] = ismember(path_edges, id_list);
    for j = path_edges_idx
        plot(edge_traj_list(j).traj(:,1), edge_traj_list(j).traj(:,2), 'Color', cmap(i,:), 'LineWidth', 3);
    end
end