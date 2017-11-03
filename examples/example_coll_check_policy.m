%% 
% Copyright (c) 2017 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%

clc;
clear;
close all;

rng(3);
%% Load data
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_2d_6/');
set_dataset = strcat(dataset,'set_1/');
do_save = false;
do_pause = false;
do_debug = false;
do_final_path = true;
do_plot = false;

G = load_graph( strcat(set_dataset,'graph.txt') );
load(strcat(set_dataset, 'world_library_assignment.mat'), 'world_library_assignment');
load(strcat(set_dataset, 'path_library.mat'), 'path_library');
%coll_check_results = dlmread( strcat(set_dataset, 'coll_check_results.txt') );
load( strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results' );

% Only for 2d viz
env_dataset = strcat(dataset,'environments/');
load( strcat(set_dataset, 'coord_set.mat'), 'coord_set'); %Optional

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
train_id = 1:size(world_library_assignment,1);
train_world_library_assignment = world_library_assignment(train_id, :);
train_coll_check_results = coll_check_results(train_id, :);

%% Select a policy
option_policy = 17;

switch(option_policy)
    case 1
        policy = policyRandomEdge(path_edgeid_map, size(train_coll_check_results, 2));
    case 2
        policy = policyMaxTallyEdge(path_edgeid_map, size(train_coll_check_results, 2));
    case 3
        policy = policyWeightedProbMaxTallyEdge(path_edgeid_map, train_world_library_assignment, train_coll_check_results, 0.2, true);
    case 4
        policy = policyRandomPathRandomEdge(path_edgeid_map, size(train_coll_check_results, 2));
    case 5
        policy = policyRandomPathMaxTallyEdge(path_edgeid_map, size(train_coll_check_results, 2));
    case 6
        policy = policyMaxProbPathRandomEdge(path_edgeid_map, train_world_library_assignment, train_coll_check_results, 0.01, false);
    case 7
        policy = policyMaxTallyEdge(path_edgeid_map, train_world_library_assignment, train_coll_check_results, 0.01, false, 1);
    case 8 %not realizable
        policy = policyDRDonevall(train_world_library_assignment, train_coll_check_results, edge_check_cost, path_edgeid_map);
    case 9 % not realizable
        policy = policyIncDRD(train_world_library_assignment, train_coll_check_results, edge_check_cost, path_edgeid_map, 5);
    case 10 %BISECT
        policy = policyDRDBernoulli(path_edgeid_map, edge_check_cost, train_world_library_assignment, train_coll_check_results, 0.01, false, 1);
    case 11
        policy = policyDRDandBern(train_world_library_assignment, train_coll_check_results, edge_check_cost, path_edgeid_map, 0.01, false, 0.2);
    case 12 
        policy = policyIncDRDandBern(train_world_library_assignment, train_coll_check_results, edge_check_cost, 5, path_edgeid_map, 0.01, false, 0.2);
    case 13 % DIRECT
        load(strcat(set_dataset, 'saved_decision_trees/drd_decision_tree_data.mat'), 'decision_tree_data');
        policy = policyDecisionTreeandBern(decision_tree_data, path_edgeid_map, edge_check_cost, 0.01, false, 0.2);
    case 14
        policy = policyMaxMVOI(path_edgeid_map, train_world_library_assignment, train_coll_check_results, 0.01, false); 
    case 15
        policy = policyMaxSetCover(path_edgeid_map, train_world_library_assignment, train_coll_check_results, 0.01, false, 1);
    case 16 %LAZYSP
        policy = policyLazySP(path_edgeid_map, train_world_library_assignment, train_coll_check_results, G, 0.01, 0);
    case 17 % DIRECT (Direct)
        load(strcat(set_dataset, 'saved_decision_trees/drd_decision_tree_data.mat'), 'decision_tree_data');
        policy = policyDecisionTreeOnly(decision_tree_data, path_edgeid_map, edge_check_cost, 0.01, false);
end

%% Perform stuff
test_world = test_id(1); %just a random world picked from test set with a guarantee that it has a path feasible

% 2d visualization
load(strcat(env_dataset, 'world_',num2str(test_world),'.mat'), 'map');
figure(1); plot_map_graph(map, G, coord_set); pause();

selected_edge_outcome_matrix = [];
path_id = [];
while (1)
    selected_edge = policy.getEdgeToCheck(); % Call policy to select edge
    if (isempty(selected_edge))
        error('No valid selection made'); % Invalid selection made
    end
    
    outcome = coll_check_results(test_world, selected_edge); %Observe outcome
    fprintf('Selected edge : %d Outcome : %d \n', selected_edge, outcome);
    
    policy.printDebug();
    
    if (do_plot)
    figure(1); 
    cla; 
    visualize_map(map); 
    view_graph( G, coord_set ); 
    if (do_debug)
        policy.plotDebug2D(G, coord_set, path_library); 
    end
    plot_map_graph_edge_outcome(map, G, coord_set, selected_edge_outcome_matrix);
    
    if (do_pause)
        pause();
    else
        pause(0.1);
    end
    if (do_save)
        saveas(gcf, strcat(num2str(size(selected_edge_outcome_matrix,1)+1),'.pdf'));
    end
    end
    
    selected_edge_outcome_matrix = [selected_edge_outcome_matrix; selected_edge outcome]; %Update event matrixx
    policy.setOutcome(selected_edge, outcome); %Set outcome to policy
        
    [done, path_id] = any_path_feasible( path_edgeid_map, selected_edge_outcome_matrix );
    if (done)
        break;
    end
    
end

%%
figure(1); 
cla; 
visualize_map(map); 
view_graph( G, coord_set ); 
if (do_debug)
    policy.plotDebug2D(G, coord_set, path_library); 
end
plot_map_graph_edge_outcome(map, G, coord_set, selected_edge_outcome_matrix);
if (do_final_path)
    plot_path( path_library{path_id}, coord_set, 'm', 4 );
end
if (do_save)
    saveas(gcf, 'final.pdf');
end

fprintf('Num edges checked: %d Cost of check: %f \n', size(selected_edge_outcome_matrix, 1), sum(edge_check_cost(selected_edge_outcome_matrix(:,1))));
