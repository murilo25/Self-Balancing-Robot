% Fixed inverted pendulum control using feedback linearization & PD
% control. It also has a Kalman Filter implementation for filtering 
% measurement noise.

clc;
clear all;
close all;

%% Define model parameters (all in SI)
global dt L m g J mu_theta var_theta mu_theta_dot var_theta_dot max_tau
dt = 0.01;
L = 1;
m = 0.2;
g = 9.8;
J = (m*L^2)/3;
max_tau = 10;
%% sensor parameters
mu_theta = 0;
var_theta = 2 * pi/180; % 2 degrees
mu_theta_dot = 0;
var_theta_dot = 1 * pi/180; % 1 degree/s
%% EKF filter parameters
global Q R
P = 1000*eye(2);
R = [var_theta^2 0 ; 0 var_theta_dot^2]; %[2 * pi/180 0 ; 0 1*pi/180];    % measurement noise
Q = [0 0 ; 0 0];  % process noise
%% Define PD controller gains
global Kp Kd
Kp = 25;
Kd = 0; % Kd makes system oscillate

%% Define simulation parameters
t_sim = 1;
k = 1;  % k-th sample
n_samples = t_sim/dt;

%% Initialize variables
%ground_truth = zeros(n_samples,2);
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
theta(1) = pi/3;
theta_dot(1) = 0;
x(1) = L*cos(pi/2 - theta(1));
y(1) = L*sin(pi/2 - theta(1));
states = [theta(1);theta_dot(1);x(1);y(1)];

%% Define desired states
theta_target = 0;
theta_target_dot = 0;

%% Start simulation ground truth
while k*dt <= t_sim
    %% Controller (feedback linearization + PD)
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
    states = plantModel([theta(k);theta_dot(k);x(k);y(k)],tau(k));
    
    theta(k + 1) = states(1);
    theta_dot(k + 1) = states(2);
    x(k + 1) = states(3);
    y(k + 1) = states(4);
    
    t(k) = k*dt;
    k = k + 1;  % update to next step
end
ground_truth = [theta theta_dot];

%% Start simulation noisy
k = 1;
while k*dt <= t_sim
    %% Controller (feedback linearization + PD)
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
    %states = plantModel([theta(k);theta_dot(k);x(k);y(k)],tau(k));
    states = plantModel([theta(k);theta_dot(k);x(k);y(k)],tau(k));
    
    theta(k + 1) = states(1);
    theta_dot(k + 1) = states(2);
    x(k + 1) = states(3);
    y(k + 1) = states(4);
    
    %% add measurement noise
    sensor1 = theta(k + 1) + ( mu_theta + var_theta*randn(1) );  % theta
    sensor2 = theta_dot(k + 1) + ( mu_theta_dot + var_theta_dot*randn(1) ); % theta_dot
    measurement = [sensor1 ; sensor2];
    
    theta(k + 1) = sensor1;
    theta_dot(k + 1) = sensor2;
    
    t(k) = k*dt;
    k = k + 1;  % update to next step
end
noisy_states = [theta theta_dot];

%% Start simulation filtered
k = 1;
while k*dt <= t_sim
    %% Controller (feedback linearization + PD)
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
    states = plantModel([theta(k);theta_dot(k);x(k);y(k)],tau(k));
    
    theta(k + 1) = states(1);
    theta_dot(k + 1) = states(2);
    x(k + 1) = states(3);
    y(k + 1) = states(4);
    
    %% add measurement noise
    sensor1 = theta(k + 1) + ( mu_theta + var_theta*randn(1) );  % theta
    sensor2 = theta_dot(k + 1) + ( mu_theta_dot + var_theta_dot*randn(1) ); % theta_dot
    measurement = [sensor1 ; sensor2];
    
    theta(k + 1) = sensor1;
    theta_dot(k + 1) = sensor2;

    %% EKF filter
    [x_hat,P_hat] = ekfPredict([theta(k) ; theta_dot(k)],measurement,P);
    
    theta(k + 1) = x_hat(1);
    theta_dot(k + 1) = x_hat(2);
    P = P_hat;
    
    t(k) = k*dt;
    k = k + 1;  % update to next step
end
estimated_states = [theta theta_dot];
t = [0 ; t];

%% Plot trajectory
figure
plot(t',ground_truth(:,1)*180/pi,'b','LineWidth',2)   % plot in degrees
hold on
plot(t',noisy_states(:,1)*180/pi,'ko','LineWidth',1)
hold on
plot(t',estimated_states(:,1)*180/pi,'r:','LineWidth',2)
ylabel('Theta [ degrees]')
xlabel('time [s]')
legend('ground truth','noisy','filtered')

figure
plot(t',ground_truth(:,2),'b','LineWidth',2)   % plot in degrees
hold on
plot(t',noisy_states(:,2),'ko','LineWidth',1)
hold on
plot(t',estimated_states(:,2),'r:','LineWidth',2)
ylabel('Angular velocity [rad/s]')
xlabel('time [s]')
legend('ground truth','noisy','filtered')

function [states_new] = plantModel(states,u)

    global L m g J dt

    theta = states(1);
    theta_dot = states(2);
    x = states(3);
    y = states(4);
    tau = u;
    
    % assume pendulum range from -pi/2 to pi/2 with
    % reference being the vertical axis
    max_range = pi/2;
    min_range = -pi/2;
    
    x_new = L*cos(pi/2 - theta);
    y_new = L*sin(pi/2 - theta);  
    if ( y_new < 0 )       % limit motion according assumption
        y_new = 0;         % limit y coordinate to 0
    end

    theta_new = theta + theta_dot*dt;
    if ( theta_new > max_range )    % limit range to 
        theta_new = max_range;
    elseif ( theta_new < min_range )
            theta_new = min_range;
    end
    theta_dot_new = (1/J)*L*m*g*sin(theta) - (1/J)*tau;
    % Limit angular velocity when pendulum range limits are violated
    if ( (y_new <= 0) || (theta_new >= max_range) || (theta_new <= min_range) )
        theta_dot_new = 0;
    end
    
    states_new = [theta_new theta_dot_new x_new y_new];
    
end

function [x_hat,P_hat] = ekfPredict(x,z,P)
    global dt Kp Kd Q R
    
    F = [1 dt ; -Kp -Kd]; % Jacobian matrix (linearization around x)
    
    H = eye(2); % identity since the state variables are the outputs of the system
    
    % prediction
    x_pred = F*x;
    P_pred = F*P*F' + Q;
    % measurement update
    z_pred = H*x_pred;    
    y = z - z_pred;
    S = H*P_pred*H' + R;
    K = P_pred*H'*S^(-1);
    x_hat = x_pred + K*y;
    P_hat = (eye(2) - K*H)*P_pred;
    
%     while (x_hat(1) > pi)
%         x_hat(1) = x_hat(1) -  2 * pi;
%     end
%     while (x_hat(1) < -pi)
%         x_hat(1) = x_hat(1) + 2 * pi;
%     end
    
end