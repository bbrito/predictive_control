function x_next =  dynamic_scenario( z, p )
%            inputs               |               states
%                acc   delta  sv     x      y       psi   v    s    dummy
    stepSize = 0.4;
    % ego-vehicle
    x = z(4: 7);
    u = z(1: 2);
    % integrator Runge-Kutta integrator of order 4
    x_R_next = RK4(x, u, @continuous_dynamics_R, stepSize);
    x_next = x_R_next;
end

function xdot = continuous_dynamics_R ( x, u )
    v = u(1);
    w = u(2);
    theta =  x(3);

    xdot = [v * cos(theta);
            v * sin(theta);
            w;
            v];
end