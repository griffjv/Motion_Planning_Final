%% Housekeeping
clear all;clc;close all;

%% Obstacles
num_objects = 5;
h_vals = [60, 34, 54.3, 100.0, 42.1];
w_vals = [68.2, 12.3, 14.5, 83, 73.2];
d_vals = [19.4, 12, 16.3, 85.3,43];


x_vals = [1, 15, 30, 45, 20];
y_vals = [1, 15, 30, 45, 60];
z_vals = [0, 0, 0, 0, 0];

%% Plot environment
permute_mat = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
figure
hold on
grid on
for i = 1:num_objects
    edges = [w_vals(i) d_vals(i) h_vals(i)];
    origin = [x_vals(i) y_vals(i) z_vals(i)];
    plotcube(edges, origin,.75, 'b')
end

workspace_bounds = [0 0 0;150 0 0;150 150 0;0 150 0];
fill3(workspace_bounds(:,1), workspace_bounds(:,2), workspace_bounds(:,3), [0.4660 0.6740 0.1880])

%% Plot path
path = processPath('goal1path.txt', 3);
edges = [5 5 1];
for i = 1:length(path(:,1))
    plotcube(edges, path(i,:),1, 'r')
end
plot3(path(:,1),path(:,2),path(:,3), 'r', 'linewidth', 2)

%% Plot goal region
goal_region = [20 30 0;30 30 0;30 40 0;20 40 0];
fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')

goal_region = [40 21 0;50 21 0;50 30 0;40 30 0;];
fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')