# Learning Collision Checking Policies
Official repository for evaluating collision checking policies that use bayesian active learning techniques

## Related Publications
1.[Near-Optimal Edge Evaluation in Explicit Generalized Binomial Graphs (NIPS 2017)](https://arxiv.org/pdf/1706.09351.pdf)

## Datasets
The repository containing datasets is [graph_collision_checking_dataset](https://github.com/sanjibac/graph_collision_checking_dataset)

## Setup
1. Clone the repository and the datasets folder
2. cd `matlab_learning_collision_checking`
3. Edit init_setup.m to add the global path to the datasets folder by editing
`setenv('collision_checking_dataset_folder','/path/to/data')`
4. Run install_dependencies.m
5. Run init_setup.m

## Executing the algorithms

Run the file `src/benchmark_coll_check_policy.m` to execute the algorithms in the paper on the datasets
