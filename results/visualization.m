%% Housekeeping
clear all;clc;close all;

isPathGeometric = false;
process_multi = false;
plot_energy = true;
num_solutions = 5;

figure
hold on
grid on
axis equal

% For legend
plot(-500,-500,'sc','MarkerFaceColor','c','MarkerSize',10)
plot(-505,-500,'sg','MarkerFaceColor','g','MarkerSize',10)
% plot(-500,-500,'r','linewidth',2)
plot(-500,-500,'sr','MarkerFaceColor','r','MarkerSize',10)
% plot(-500,-500, 'r', 'linewidth', 2)
% plot(-505,-500, 'Color', [0.8500 0.3250 0.0980], 'linewidth', 2)
% plot(-500,-500, 'Color', [0.3010 0.7450 0.9330], 'linewidth', 2)
% plot(-500,-500,'Color',[0 .4470 .7410], 'linewidth', 2)
xlim([-10 260])
ylim([-10 260])
%% Obstacles
num_objects = 15;
h_vals = [60, 34, 54.3, 150.0, 42.1, 69, 30, 87, 94, 60, 128, 59, 142, 72, 49];
w_vals = [68.2, 20, 24, 83, 73.2, 31, 49, 47, 59, 31, 35, 57, 48, 41, 21];
d_vals = [19.4, 20, 16.3, 85.3,43, 59, 29, 27, 50, 70, 55, 53, 75, 37, 25];


x_vals = [1, 15, 30, 45, 170, 10, 200, 180, 120, 43, 86, 119, 152, 164, 125];
y_vals = [1, 15, 30, 45, 20, 145, 200, 105, 120, 93, 23, 25, 44, 143, 210];
z_vals = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];


%% Plot path
edges = [2 2 1];

if isPathGeometric
    path = processPath('geo_result_goal1.txt', 3);
    if process_multi
       for j = 1:7
          filename = strcat('geo_result_goal', int2str(j), '.txt');
          path = [];
          path = processPath(filename, 3);
          for i = 1:length(path(:,1))
          %     plotcube(edges, path(i,1:3),1, 'r')
          end
          plot3(path(:,1),path(:,2),path(:,3), 'r', 'linewidth', 2)
       end
    else
        path = processPath('geo_result_goal1.txt', 3);
        for i = 1:length(path(:,1))
        %     plotcube(edges, path(i,1:3),1, 'r')
        end
        plot3(path(:,1),path(:,2),path(:,3), 'r', 'linewidth', 2)        
    end
else
    prop_timestep = 0.1;
    if process_multi
        for j = 1:num_solutions
%           filename = strcat('kino_result_goal', int2str(j), '.txt');
          filename = strcat('kinodynamic_SSTresult', int2str(j),'.txt');
          path = [];
          path = processKinodynamicPath(filename, false);
          for i = 1:length(path(:,1))
          %     plotcube(edges, path(i,1:3),1, 'r')
          end
          plot3(path(:,1),path(:,2),path(:,3), 'r', 'linewidth', 2)
        end
    else
          filename = 'kinodynamic_SSTresult.txt';
          path = processKinodynamicPath(filename, false);
          plot3(path(:,1),path(:,2),path(:,3), 'r', 'linewidth', 2)
    end
end

%% Plot environment
permute_mat = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
for i = 1:num_objects
    edges = [w_vals(i) d_vals(i) h_vals(i)];
    origin = [x_vals(i) y_vals(i) z_vals(i)];
    plotcube(edges, origin,.75, 'b')
end

workspace_bounds = [0 0 0;250 0 0;250 250 0;0 250 0];
fill3(workspace_bounds(:,1), workspace_bounds(:,2), workspace_bounds(:,3), [0.4660 0.6740 0.1880])


%% Plot goal regions
% goal_region = [62 22 0;78 22 0;78 38 0;62 38 0]; %x = 70, y = 30
% fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')
% 
% goal_region = [132 90 0;148 90 0;148 108 0;132 108 0]; %x = 140, y = 100
% fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')
% 
% goal_region = [217 77 0;233 77 0;233 93 0;217 93 0]; %x = 225, y = 85
% fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')
% 
% goal_region = [17 217 0;33 217 0;33 233 0;17 233 0]; %x = 25, y = 225
% fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')
% 
% goal_region = [77 142 0;93 142 0;93 158 0;77 158 0]; %x = 85, y = 150
% fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')
% 
% goal_region = [12 92 0;28 92 0;28 108 0;12 108 0]; %x = 20, y = 100
% fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')
% 
% goal_region = [207 152 0;223 152 0;223 168 0;207 168 0]; %x = 215, y = 160
% fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')

goal_region = [62-5 22-5 0;78+5 22-5 0;78+5 38+5 0;62-5 38+5 0]; %x = 70, y = 30
fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')

goal_region = [132-5 90-5 0;148+5 90-5 0;148+5 108+5 0;132-5 108+5 0]; %x = 140, y = 100
fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')

goal_region = [217-5 77-5 0;233+5 77-5 0;233+5 93+5 0;217-5 93+5 0]; %x = 225, y = 85
fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')

goal_region = [17-5 217-5 0;33+5 217-5 0;33+5 233+5 0;17-5 233+5 0]; %x = 25, y = 225
fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')

goal_region = [77-5 142-9 0;93+5 142-9 0;93+5 158+5 0;77-5 158+5 0]; %x = 85, y = 150
fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')

goal_region = [12-5 92-5 0;28+5 92-5 0;28+5 108+5 0;12-5 108+5 0]; %x = 20, y = 100
fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')

goal_region = [207-5 152-5 0;223+5 152-5 0;223+5 168+5 0;207-5 168+5 0]; %x = 215, y = 160
fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'g')


%% Plot start region
goal_region = [235 235 0;245 235 0;245 245 0;235 245 0]; %x = 215, y = 160
goal_region = [230 230 0;250 230 0;250 250 0;230 250 0]; %x = 215, y = 160

fill3(goal_region(:,1), goal_region(:,2), goal_region(:,3),'c')

%% Labeling
xlabel('x-direction(m)','FontSize',20)
ylabel('y-direction(m)','FontSize',20)
zlabel('z-direction(m)','FontSize',20)
legend('Starting Region', 'Goal Regions', 'UAV Trajectory', 'Environment Obstacles', 'FontSize', 14)
legend('Refueling Regions', 'Gathering Regions', 'Transmitting Regions', 'FontSize', 14)



%% Plot energy info
figure
subplot(2,1,1)
hold on
set(gca,'FontSize',20)
grid on
for i=1:num_solutions
      filename = strcat('kinodynamic_SSTresult', int2str(i), '.txt');
      path = [];
      path = processKinodynamicPath(filename, false);
      time_vec = [];
      time_vec(1) = 0;
      for j = 2:length(path(:,end))
         time_vec(j) = time_vec(j-1) + path(j,end); 
      end
      
      plot(time_vec',path(:,9),'linewidth',2)
end
xlabel('Time(s)','FontSize',20)
ylabel('Thrust(N/kg)','FontSize',20)
title('Thrust Force vs Path Duration for Intermediate and Final SST Trajectories','FontSize',18)



subplot(2,1,2)
hold on
set(gca,'FontSize',20)
grid on
for i=1:num_solutions
      filename = strcat('kinodynamic_SSTresult', int2str(i), '.txt');
      path = [];
      path = processKinodynamicPath(filename, false);
      time_vec = [];
      time_vec(1) = 0;
      for j = 2:length(path(:,end))
         time_vec(j) = time_vec(j-1) + path(j,end); 
      end
      energy = [];
      energy(1) = 0;
      for k = 2:length(path(:,end))
        energy(k) = (path(k-1,9)+path(k,9))/2 * path(k,end) + energy(k-1);
      end
      plot(time_vec',energy,'linewidth',2)
      plot(time_vec(end),energy(end),'sr','MarkerFaceColor','r','MarkerSize',14)
      energy(end)
end
xlabel('Time(s)','FontSize',20)
ylabel('Energy','FontSize',20)
title('Energy Cost Accumulation vs Path Duration for Intermediate and Final SST Trajectories','FontSize',18)