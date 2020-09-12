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
L = 10;
m = 2;
g = 9.8;
J = (m*L^2)/3;
max_tau = 100;

%% Set initial conditions
x0 = [-0.0625 ; 0];  % stable for initial angles in between 0.-625 and 0.625
t_sim = 0:dt:10;

%% Solve dynamics
[t,x] = ode45(@(t,x) FixedInvertedPend(t,x,L,m,g,J,max_tau),t_sim,x0);

%% Plot trajectory
subplot(2,1,1)
plot(t,x(:,1))
hold on
plot(t,x(:,2))
title('State trajectories')
ylabel('states')
xlabel('time [s]')
legend('angle','angular velocity')

subplot(2,1,2)
plot(ode_t,tau_history)
ylabel('control input')
xlabel('time [s]')
