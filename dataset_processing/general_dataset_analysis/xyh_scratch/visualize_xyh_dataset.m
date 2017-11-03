clc;
clear;
close all;

rng(3);
%% Load data
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_xyh_1/');
set_dataset = strcat(dataset,'set_1/');

G = load_graph( strcat(set_dataset,'graph.txt') );
load(strcat(set_dataset, 'world_library_assignment.mat'), 'world_library_assignment');
load(strcat(set_dataset, 'path_library.mat'), 'path_library');
path_library = path_library(1:20);
load( strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results' );

% Only for 2d viz
env_dataset = strcat(dataset,'environments/');
load(strcat(set_dataset, 'edge_traj_list.mat'), 'edge_traj_list');
id_list = sub2ind(size(G), [edge_traj_list.id1]', [edge_traj_list.id2]');

%% Extract relevant info
world_library_assignment = logical(world_library_assignment);
coll_check_results = logical(coll_check_results);
edge_check_cost = ones(1, size(coll_check_results,2)); %transpose(full(G(find(G)))); %
path_edgeid_map = get_path_edgeid_map( path_library, G );

%% Do a dimensionality reduction
if(isequal(tril(G), triu(G)))
    % Then its undirected and we assume the path forward is the path back
    % and can just check lower triangle of G leading to huge savings
    [ G, coll_check_results, edge_check_cost, path_edgeid_map ] = remove_redundant_edges( G,coll_check_results, edge_check_cost, path_edgeid_map  );
end

%% Load train test id
load(strcat(set_dataset, 'train_id.mat'), 'train_id');
load(strcat(set_dataset, 'test_id.mat'), 'test_id');

train_world_library_assignment = world_library_assignment(train_id, :);
train_coll_check_results = coll_check_results(train_id, :);

%% Perform stuff
test_world = test_id(17); %just a random world picked from test set with a guarantee that it has a path feasible

% 2d visualization
load(strcat(env_dataset, 'world_',num2str(test_world),'.mat'), 'map');
figure(1);
hold on;
visualize_map(map);

status = G;
status(find(status)) = status(find(status)).*transpose(coll_check_results(test_world,:));

coord_set = [];
edge_traj_list_new = [];
for i = 1:size(edge_traj_list,1)
    if (status(edge_traj_list(i).id1, edge_traj_list(i).id2) ~= 0)
        coord_set = [coord_set;
            edge_traj_list(i).traj(1,1) edge_traj_list(i).traj(1,2);
            edge_traj_list(i).traj(end,1) edge_traj_list(i).traj(end,2)];
        edge_traj_list_new = [edge_traj_list_new;edge_traj_list(i)];
    end
end
plot_edge_traj_list( edge_traj_list_new, [0.8 0.8 0.8] );
scatter(coord_set(:,1), coord_set(:,2),30,'k', 'filled');

