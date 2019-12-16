function x_next =  dynamic_scenario( z, p, w  )
%            inputs               |               states
%                acc   delta  sv     x      y       psi   v    s    dummy
    stepSize = 0.1;
    % ego-vehicle
    x = zeros(6, 1);
    x = z(4: 9);
    u = zeros(3, 1);
    u = z(1: 3);
    % integrator Runge-Kutta integrator of order 4
    x_R_next = RK4(x, u, @continuous_dynamics_R, stepSize, 3.0);
    x_next = x_R_next;
end

function xdot = continuous_dynamics_R ( x, u, L )
    a = u(1);
    delta = u(2);
    sv = u(3);
    psi =  x(3);
    v = x(4);
    lr = 1.5;
    lf = 1.5;
    ratio = lr/(lr + lf);
    beta = atan(ratio*tan(delta));
    xdot = [v * cos(psi + beta);
            v * sin(psi + beta);
            (v / lr) * sin(beta);
            a;
            v;
            sv];
end