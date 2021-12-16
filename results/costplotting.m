%% housekeeping
clear all;clc;close all;

%% cost
figure
hold on
grid on
costs = [489.57 486.36 486.26 485.91];
plot(costs,'-*b','linewidth',2)
costs = [536.29 496 426.7886 399.7690];
plot(costs,'-*b','linewidth',2)
costs = [523.64 291.81];
plot(costs,'-*b','linewidth',2)
costs = [174.69 172.13 170.09];
plot(costs,'-*b','linewidth',2)
costs = [389.08 381.44 380.61 380.07 379.32 378.98 376.57];
plot(costs,'-*b','linewidth',2)
costs = [526.24 223.18];
plot(costs,'-*b','linewidth',2)
costs = [750.49 745.95];
plot(costs,'-*b','linewidth',2)

costs = [244.76 241.56 239.09 238.95 238.91 238.90 238.88 238.86];
plot(costs,'-*b','linewidth',2)

costs = [372.32 370.53 370.51];
plot(costs,'-*b','linewidth',2)

costs = [599.57 446.02];
plot(costs,'-*b','linewidth',2)

costs = [380.97 371.88 266.14];
plot(costs,'-*b','linewidth',2)

xlabel('Solution Iteration','FontSize',20)
ylabel('Total Energy Along Path','FontSize',20)
set(gca,'FontSize',20)