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

%% Define model parameters (all in SI)
dt = 0.01;
L = 1;
m = 0.2;
g = 9.8;
J = (m*L^2)/3;
max_tau = 10;
% sensor parameters
mu_theta = 0;
var_theta = 2 * pi/180; % 2 degrees
mu_theta_dot = 0;
var_theta_dot = 1 * pi/180; % 1 degree/s

%% Define simulation parameters
t_sim = 1;
k = 1;  % k-th sample
n_samples = t_sim/dt;

%% Initialize variables
ground_truth = zeros(n_samples,2);
x = zeros(n_samples,1);
y = zeros(n_samples,1);
theta = zeros(n_samples,1);
theta_dot = zeros(n_samples,1);
e = zeros(n_samples,1);
e_dot = zeros(n_samples,1);
e_acc = zeros(n_samples,1);
tau = zeros(n_samples,1);
t = zeros(n_samples,1);

%% Set initial conditions
theta(1) = pi/3;    % stable for initial angles in between -0.625 and 0.625
theta_dot(1) = 0;
x(1) = L*cos(pi/2 - theta(1));
y(1) = L*sin(pi/2 - theta(1));
ground_truth(1,:) = [theta(1) theta_dot(1)];

%% Define PD controller gains
Kp = 25;
Kd = 0; % Kd makes system oscillate

%% Define desired states
theta_target = 0;
theta_target_dot = 0;

%% Start simulation
while k*dt <= t_sim
    %% Controller 
    % Define error
    e(k) = theta_target - theta(k);
    e_dot(k) = theta_target_dot - theta_dot(k);
    % Compute control input
    tau(k) = L*m*g*sin(theta(k)) - J*(Kd*e_dot(k) + Kp*e(k));
    % saturate control input
    if (tau(k) > max_tau) 
        tau(k) = max_tau;
    elseif (tau(k) < -max_tau)
        tau(k) = -max_tau;
    end

    %% Plant dynamics: 
    % assume pendulum range from -pi/2 to pi/2 with
    % reference being the vertical axis
    x(k+1) = L*cos(pi/2 - theta(k));
    y(k+1) = L*sin(pi/2 - theta(k));  
    if ( y(k+1) < 0 )       % limit motion according assumption
        y(k+1) = 0;         % limit y coordinate to 0
    end

    theta(k + 1) = theta(k) + theta_dot(k)*dt;
    if ( theta(k+1) > pi/2 )    % limit range to 
        theta(k+1) = pi/2;
    elseif ( theta(k+1) < - pi/2 )
            theta(k+1) = -pi/2;
    end
    theta_dot(k + 1) = (1/J)*L*m*g*sin(theta(k)) - (1/J)*tau(k);
    % Limit angular velocity when pendulum range limits are violated
    if ( (y(k+1) < 0) || (theta(k+1) > pi/2) || (theta(k+1) < -pi/2) )
        theta_dot(k+1) = 0;
    end
    
    ground_truth(k + 1,:) = [theta(k + 1) theta_dot(k + 1)];
    % add sensor noise
    theta(k + 1) = theta(k + 1) + ( mu_theta + var_theta*randn(1) );
    theta_dot(k + 1) = theta_dot(k + 1) + ( mu_theta_dot + var_theta_dot*randn(1) );
    
    t(k) = k*dt;
    k = k + 1;  % update to next step
end
t = [0 ; t];
%% Plot trajectory
subplot(2,1,1)
plot(t',theta*180/pi)   % plot in degrees
hold on
plot(t',theta_dot)
hold on
plot(t',(180/pi)*ground_truth(:,1),'b:')
hold on
plot(t',ground_truth(:,2),'r:')
title('State trajectories')
ylabel('states')
xlabel('time [s]')
legend('angle [degrees]','angular velocity [rad/s]','Real angle','Real angular velocity')

subplot(2,1,2)
plot(x,y,'o')
hold on
plot(x(1),y(1),'g*','LineWidth',2)
hold on
plot(x(end),y(end),'m^','LineWidth',2)
ylabel('y coordinate')
xlabel('x coordinate')
legend('Trajectory of mass','Starting location','Final location')

figure
plot(t(1:end-1)',tau)
ylabel('control input')
xlabel('time [s]')
legend('Applied torque')



