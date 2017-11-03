clc;
clear;
close all;

%% Load data
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_2d_5/');
set_dataset = strcat(dataset,'set_1/');

G = load_graph( strcat(set_dataset,'graph.txt') );
load(strcat(set_dataset, 'world_library_assignment.mat'), 'world_library_assignment');
load(strcat(set_dataset, 'path_library.mat'), 'path_library');
load( strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results' );

% Only for 2d viz
env_dataset = strcat(dataset,'environments/');
load( strcat(set_dataset, 'coord_set.mat'), 'coord_set'); %Optional

%% Extract relevant info
world_library_assignment = logical(world_library_assignment);
coll_check_results = logical(coll_check_results);
test_cost = ones(1, size(coll_check_results,2)); %transpose(full(G(find(G)))); %
path_edgeid_map = get_path_edgeid_map( path_library, G );

%% Some parsing
hyp_test = coll_check_results;
hyp_region = world_library_assignment;
region_test = false(length(path_edgeid_map), length(test_cost));
for i = 1:length(path_edgeid_map)
    for e = path_edgeid_map{i}
        region_test(i, e) = true;
    end
end

test_bias = (1/size(hyp_test,1))*sum(hyp_test, 1);
region_status = get_region_status( [], region_test, test_bias );
region_tally = sum(region_test, 2);

%% Plot
figure(1);
plot_edge_likelihood_regions_only( test_bias, G, coord_set, region_test );

%% Top k shortest paths
figure(2);
k = 2;
hold on;
[~, path_idx] = sort(region_tally);
cc = parula(k);
fprintf('Top shortest paths \n');
for i = 1:k
    plot_path( path_library{path_idx(i)}, coord_set, cc(i,:), 4 );
    fprintf('Num edges: %d Probability: %f \n', region_tally(path_idx(i)), region_status(path_idx(i)));
end
view_graph( G, coord_set );

%% Top k probable paths
figure(3);
hold on;
[~, path_idx] = sort(region_status, 'descend');
cc = parula(k);
fprintf('Top shortest paths \n');
for i = 1:k
    plot_path( path_library{path_idx(i)}, coord_set, cc(i,:), 4 );
    fprintf('Num edges: %d Probability: %f \n', region_tally(path_idx(i)), region_status(path_idx(i)));
    test_bias(region_test(path_idx(i),:))
end
view_graph( G, coord_set );
