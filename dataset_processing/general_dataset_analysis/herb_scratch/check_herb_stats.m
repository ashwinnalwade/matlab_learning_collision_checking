%% Create regions from graphs and coll check results
% Reads a graph
clc;
clear;
close all;

%% Load stuff
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_herb_1/');
set_dataset = strcat(dataset,'raw/');

G = load_graph( strcat(set_dataset,'graph.txt') );

load( strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results' );
load(strcat(set_dataset, 'graph_translator.mat'), 'vertex_id_map' ,'edge_id_map');
%% Check status
if (0)
    candidate_start_goal_pair_original  = ...
        [9216 9529;
        9216 6793;
        9216 6799;
        9216 271;
        2688 6793;
        2688 6799;
        2688 271];
    
    candidate_start_goal_pair = [vertex_id_map(candidate_start_goal_pair_original(:,1)) vertex_id_map(candidate_start_goal_pair_original(:,2))];
    
    [test_idx] = randperm(size(coll_check_results,1), 100);
    stats = zeros(size(candidate_start_goal_pair, 1), 1);
    parfor idx = 1:size(candidate_start_goal_pair,1)
        start_idx = candidate_start_goal_pair(idx,1);
        goal_idx = candidate_start_goal_pair(idx,2);
        success = 0;
        for i = test_idx
            fprintf('Pair: %d Test: %d \n', idx, i);
            status = G;
            status(find(status)) = status(find(status)).*transpose(coll_check_results(i,:));
            [~, path] = graphshortestpath(status, start_idx, goal_idx);
            if (~isempty(path))
                success = success + 1;
            end
        end
        stats(idx) = success;
    end
end

if (1)
    %% Path library status check
    [test_idx] = randperm(size(coll_check_results,1), 100);
    start_idx = vertex_id_map(9216);
    goal_idx = vertex_id_map(6793);
    k = 10;
    path_library = get_kshortestpaths_dataset( coll_check_results(1,:), G, start_idx, goal_idx, k );
    fprintf('Num library set before pruning: %d \n',length(path_library));
    % Apply conseqopt to prune library to desired quantity
    %num_library = Inf; %10;
    %path_library = greedily_prune_library( path_library, coll_check_results(test_idx,:), G, num_library );
    %fprintf('Num library set after pruning: %d \n',length(path_library));
    
    %%
  %  world_library_assignment = get_world_library_assignment( path_library2, coll_check_results(test_idx,:), G );
  %  fprintf('Num regions: %d Num unsolved worlds: %d \n', size(world_library_assignment, 2), nnz(~any(world_library_assignment,2)));
end

if (0)
    load path_library_big.mat path_library;
    world_library_assignment = get_world_library_assignment( path_library, coll_check_results, G );
    membership = (sum(world_library_assignment,1))/size(world_library_assignment, 1);
    candidate_paths = find(membership > 0.25 & membership < 0.75);
    [~, ia] = unique(world_library_assignment', 'rows');
    candidate_paths = intersect(candidate_paths, ia);
    path_library = path_library(candidate_paths);
    world_library_assignment = get_world_library_assignment( path_library, coll_check_results, G );
    fprintf('Num regions: %d Num unsolved worlds: %d \n', size(world_library_assignment, 2), nnz(~any(world_library_assignment,2)));
    save('path_library.mat', 'path_library');
    save( 'world_library_assignment.mat', 'world_library_assignment');
end