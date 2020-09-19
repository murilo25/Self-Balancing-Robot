function [xdot] = FixedInvertedPend(t,x,L,m,g,J,max_tau)

    global tau_history iter ode_t

    %% Define desired states
    theta_target = 0;
    theta_target_dot = 0;

    %% Define controller gains
    Kp = 10;
    Kd = 10;

    %% Define error
    e = theta_target - x(1);
    e_dot = theta_target_dot - x(2);
    %% Compute control input
    tau = L*m*g*x(1) - J*(Kd*e_dot + Kp*e);
    % saturate control input
    if (tau > max_tau) 
        tau = max_tau;
    elseif (tau < -max_tau)
        tau = -max_tau;
    end
    
    %% Store control input and time history
    iter = iter + 1;
    tau_history(iter) = tau;
    ode_t(iter) = t;

    xdot = [x(2) ; L*m*g/J*x(1) - (1/J)*tau];

end

