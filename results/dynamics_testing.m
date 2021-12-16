%% Forward Kinodynamics Testing Script
% Test dynamics model of UAV used for ASEN5519 final
% Author: Griffin Van Anne

%% housekeeping
clear all;clc;close all;

%% parameters
I = 1000; %Assume very large I to ignore rotational rate of craft
g = 9.81; %Gravitational acceleration Earth, could modify for different bodies
max_thrust = 2*g; %T/m... tune this parameter to get desired results
max_eta = pi/4;
max_rho = pi/4;

%% initial conditions
x_0 = 140;
y_0 = 140;
z_0 = 10;
x_dot_0 = 0;
y_dot_0 = 0;
z_dot_0 = 0;
eta_0 = deg2rad(10);
rho_0 = deg2rad(45);
T_0 = 9.81;
ic2d = [x_0; y_0; x_dot_0; y_dot_0; eta_0; T_0];
ic3d = [x_0; y_0; z_0; x_dot_0; y_dot_0; z_dot_0; eta_0; rho_0; T_0];
%% simulate
controls = [-.0646315 .516279 -5.0979];
duration = 2;

tspan = [0 duration];
[t, y] = ode45(@(t,y) three_d_dynamics(t,y,g,controls, max_eta,max_rho,max_thrust), tspan, ic3d);

%% visualize
figure
plot(t, y(:,2), 'linewidth', 2)
grid on
hold on
plot(t, y(:,1),'linewidth', 2)
plot(t, y(:,3),'linewidth', 2)
xlabel('Time(s)')
ylabel('Distance(m)')
legend('y dir', 'x dir', 'z dir')
