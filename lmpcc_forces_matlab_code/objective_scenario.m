function [cost] =  objective_scenario( z, p)
%% Cost for ego-vehicle    
% states and inputs for ego vehicle
%            inputs               |               states
%                v   w  sv     x      y       theta      dummy
x_R = z(4: 6);
u_R = z(1: 2);
v = u_R(1);
w = u_R(2);
sv = z(3);
x = x_R(1);
y = x_R(2);
theta = x_R(3);

% Weights
Wx = p(1);
Wy = p(2);
Wv = p(3);
Ww = p(4);

% Trajectory Ref
x_ref = p(5);
y_ref = p(6);
v_ref = p(7);

%% Total cost
error_x = x - x_ref;
error_y = y - y_ref;
cost = Wx*error_x*error_x + Wy*error_y*error_y +Ww*w*w + 10*sv*sv + Wv*v*v;

end
