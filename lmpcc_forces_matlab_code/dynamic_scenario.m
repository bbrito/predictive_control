function x_next =  dynamic_scenario( z, p, w  )
%            inputs               |               states
%                acc   delta  sv     x      y       psi   v    s    dummy
    stepSize = 0.4;
    % ego-vehicle
    x = zeros(4, 1);
    x = z(4: 7);
    u = zeros(3, 1);
    u = z(1: 3);
    % integrator Runge-Kutta integrator of order 4
    x_R_next = RK4(x, u, @continuous_dynamics_R, stepSize, 3.0);
    x_next = x_R_next;
end

function xdot = continuous_dynamics_R ( x, u, L )
    v = u(1);
    w = u(2);
    sv = u(3);
    theta =  x(3);

    xdot = [v * cos(theta);
            v * sin(theta);
            w;
            sv];
end