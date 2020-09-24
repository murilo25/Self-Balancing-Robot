%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% Self-balancing robot project         %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% First model: Fixed inverted pendulum %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% Murilo A. Pinheiro                   %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% 09/11/2020                           %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;
close all;

%% Define global variables for plotting control input
global tau_history iter ode_t
iter = 0;

%% Define model parameters (all in SI)
dt = 0.05;
L = 1;
m = 0.2;
g = 9.8;
J = (m*L^2)/3;
max_tau = 100;

%% Set initial conditions
x0 = [1.0625 ; 0];
t_sim = 0:dt:10;

%% Solve dynamics
[t,states] = ode45(@(t,x) FixedInvertedPend(t,x,L,m,g,J,max_tau),t_sim,x0);
x_coord = L*cos(pi/2 - states(:,1));
y_coord = L*sin(pi/2 - states(:,1));  
%% Plot trajectory
subplot(2,1,1)
plot(t,states(:,1))
hold on
plot(t,states(:,2))
title('State trajectories')
ylabel('states')
xlabel('time [s]')
legend('angle','angular velocity')

subplot(2,1,2)
subplot(2,1,2)
plot(x_coord,y_coord,'o')
hold on
plot(x_coord(1),y_coord(1),'g*','LineWidth',2)
hold on
plot(x_coord(end),y_coord(end),'m^','LineWidth',2)
ylabel('y coordinate')
xlabel('x coordinate')
legend('Trajectory of mass','Starting location','Final location')

figure
plot(ode_t,tau_history)
ylabel('control input')
xlabel('time [s]')
