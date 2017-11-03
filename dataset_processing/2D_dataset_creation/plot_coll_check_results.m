clc;
clear;
close all;

%% Load data
dataset = strcat(getenv('collision_checking_dataset_folder'), '/dataset_2d_1/');
set_dataset = strcat(dataset,'set_1/');
load(strcat(set_dataset, 'coll_check_results.mat'), 'coll_check_results');
G = load_graph( strcat(set_dataset, 'graph.txt') );
load(strcat(set_dataset, 'coord_set.mat'), 'coord_set');
%% Plot
edge_likelihood = (1/size(coll_check_results,1))*sum(coll_check_results,1);
figure(1);
plot_edge_likelihood(edge_likelihood, G, coord_set);