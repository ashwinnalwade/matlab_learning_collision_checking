clc;
clear;
close all;

%% Parameters
input_dataset_name = 'halton_1';
folder = '/Volumes/NO NAME/Content/datasets/matlab_learning_collision_checking_dataset_creation/herb_collision_checking_dataset_creation/';
input_graph_file_name = strcat(folder,'graphs/herb_',input_dataset_name,'_parsed.txt');
input_collision_dataset_folder_name = strcat(folder,'checkresults/onlyobject/');

output_dataset_name = strcat(folder, 'parsed_datasets/onlyobject');
output_graph_filename = strcat(output_dataset_name,'/graph.txt');
output_graph_translator_filename = strcat(output_dataset_name,'/graph_translator.mat');
output_start_goal_filename = strcat(output_dataset_name,'/start_goal.mat');
output_coll_check_results_filename = strcat(output_dataset_name,'/coll_check_results_old.mat');
%% First lets parse the graph
%num_vertex = dlmread(input_graph_file_name, ' ', [0 0 0 0]);
num_edges = dlmread(input_graph_file_name, ' ', [1 0 1 0]);
edges = dlmread(input_graph_file_name, ' ', 2, 0);
if (num_edges ~= size(edges,1))
    error('inconsistent data');
end

vertex_id = union(edges(:,2), edges(:,3));
vertex_id_map = zeros(max(vertex_id), 1);
vertex_id_map(vertex_id) = transpose(1:length(vertex_id));
num_vertex = size(vertex_id, 1);
G = sparse(num_vertex, num_vertex);

parent_vertices = vertex_id_map(edges(:,2));
child_vertices = vertex_id_map(edges(:,3));
G(sub2ind(size(G), parent_vertices, child_vertices)) = edges(:,4);
G(sub2ind(size(G), child_vertices, parent_vertices)) = edges(:,4);

%need a map from edge_id to p, c
edge_id_map = [parent_vertices child_vertices];

start_idx = vertex_id_map(9216);
goal_idx = vertex_id_map(6793);

save_graph(output_graph_filename, G );
save(output_graph_translator_filename, 'vertex_id', 'vertex_id_map' ,'edge_id_map');
save(output_start_goal_filename, 'start_idx', 'goal_idx');

%% Read collision check results
num_worlds = 1000;
coll_check_results = zeros(num_worlds, nnz(G));
G_edges = find(G);
for i = 1:num_worlds
    i
    input_collision_dataset_filename = strcat(input_collision_dataset_folder_name, 'onlyobject',num2str(i),'.txt');
    coll_status = dlmread(input_collision_dataset_filename);
    status = sparse(G);
    status(sub2ind(size(status), edge_id_map(:,1), edge_id_map(:,2))) = logical(coll_status(:,2));
    status(sub2ind(size(status), edge_id_map(:,2), edge_id_map(:,1))) = logical(coll_status(:,2));
    coll_check_results(i, :) = transpose(full(status(G_edges)));
end

%%
save(output_coll_check_results_filename, 'coll_check_results' );
