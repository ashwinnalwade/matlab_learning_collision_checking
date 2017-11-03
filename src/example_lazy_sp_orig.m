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
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_2d_5/');
set_dataset = strcat(dataset,'set_1/');
do_save = false;
do_pause = false;
do_plot = false;

G = load_graph( strcat(set_dataset,'graph.txt') );
%load(strcat(set_dataset, 'world_library_assignment.mat'), 'world_library_assignment');
%coll_check_results = dlmread( strcat(set_dataset, 'coll_check_results.txt') );
load( strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results' );
load(strcat(set_dataset, 'start_goal.mat'));

% Only for 2d viz
env_dataset = strcat(dataset,'environments/');
load( strcat(set_dataset, 'coord_set.mat'), 'coord_set'); %Optional

%% Extract relevant info
%world_library_assignment = logical(world_library_assignment);
coll_check_results = logical(coll_check_results);
edge_check_cost = ones(1, size(coll_check_results,2)); %transpose(full(G(find(G)))); %

%% Load train test id
load(strcat(set_dataset, 'train_id.mat'), 'train_id');
load(strcat(set_dataset, 'test_id.mat'), 'test_id');

%train_world_library_assignment = world_library_assignment(train_id, :);
train_coll_check_results = coll_check_results(train_id, :);

%% Policy
policy = policyLazySPOrig(G, start_idx, goal_idx);

%% Perform stuff
test_world = test_id(2); %just a random world picked from test set with a guarantee that it has a path feasible

% 2d visualization
load(strcat(env_dataset, 'world_',num2str(test_world),'.mat'), 'map');
figure(1); plot_map_graph(map, G, coord_set); pause();

selected_edge_outcome_matrix = [];
path_id = [];
while (1)
    selected_edge = policy.getEdgeToCheck(); % Call policy to select edge
    if (isempty(selected_edge))
        break;
    end
    
    outcome = coll_check_results(test_world, selected_edge); %Observe outcome
    selected_edge_outcome_matrix = [selected_edge_outcome_matrix; selected_edge outcome]; %Update event matrixx
    fprintf('Selected edge : %d Outcome : %d \n', selected_edge, outcome);
    if (do_plot)
    policy.printDebug();
    figure(1); cla; plot_map_graph_edge_outcome(map, G, coord_set, selected_edge_outcome_matrix); policy.plotDebug2D(G, coord_set, []); 
    if (do_pause)
        pause();
    else
        pause(0.1);
    end
    if (do_save)
        saveas(gcf, strcat(num2str(size(selected_edge_outcome_matrix,1)),'.pdf'));
    end
    end
    policy.setOutcome(selected_edge, outcome); %Set outcome to policy 
end

%%
figure(1); 
cla; 
visualize_map(map); 
view_graph( G, coord_set );  
plot_map_graph_edge_outcome(map, G, coord_set, selected_edge_outcome_matrix);
[~, path] = graphshortestpath(policy.Gplan, policy.start_idx, policy.goal_idx);
plot_path( path, coord_set, 'm', 4 );
if (do_save)
    saveas(gcf, 'final.pdf');
end

fprintf('Num edges checked: %d Cost of check: %f \n', size(selected_edge_outcome_matrix, 1), sum(edge_check_cost(selected_edge_outcome_matrix(:,1))));
