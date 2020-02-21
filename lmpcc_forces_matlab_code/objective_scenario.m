function [cost] =  objective_scenario( z, p, i)
%% Cost for ego-vehicle    
% states and inputs for ego vehicle
%            inputs               |               states
%                acc   delta  sv     x      y       psi   v    s    dummy
x_R = z(4: 9);
u_R = z(1: 3);
a = u_R(1);
delta = u_R(2);
sv = u_R(3);
x = x_R(1);
y = x_R(2);
v = x_R(4);
s = x_R(5);

%% Spline parameters
s01 = p(17);
s02 = p(18);
d = p(19);

a_X1 = p(1); b_X1 = p(2); c_X1 = p(3); d_X1 = p(4);
a_Y1 = p(5); b_Y1 = p(6); c_Y1 = p(7); d_Y1 = p(8);
a_X2 = p(9); b_X2 = p(10); c_X2 = p(11); d_X2 = p(12);
a_Y2 = p(13); b_Y2 = p(14); c_Y2 = p(15); d_Y2 = p(16);

% Weights
Wx = p(20);
Wy = p(21);
Wa = p(22);
Wdelta = p(23);
vref = p(24);
vref2 = p(26);
ws = p(25);
Wv = p(39);

%% Taken from Acado OCP
lambda = 1/(1 + exp((s - d)/0.1));
x_path1 = (a_X1*(s-s01)*(s-s01)*(s-s01) + b_X1*(s-s01)*(s-s01) + c_X1*(s-s01) + d_X1) ;
y_path1 = (a_Y1*(s-s01)*(s-s01)*(s-s01) + b_Y1*(s-s01)*(s-s01) + c_Y1*(s-s01) + d_Y1) ;
dx_path1 = (3*a_X1*(s-s01)*(s-s01) + 2*b_X1*(s-s01) + c_X1) ;
dy_path1 = (3*a_Y1*(s-s01)*(s-s01) + 2*b_Y1*(s-s01) + c_Y1) ;

x_path2 = (a_X2*(s-s02)*(s-s02)*(s-s02) + b_X2*(s-s02)*(s-s02) + c_X2*(s-s02) + d_X2) ;
y_path2 = (a_Y2*(s-s02)*(s-s02)*(s-s02) + b_Y2*(s-s02)*(s-s02) + c_Y2*(s-s02) + d_Y2) ;
dx_path2 = (3*a_X2*(s-s02)*(s-s02) + 2*b_X2*(s-s02) + c_X2) ;
dy_path2 = (3*a_Y2*(s-s02)*(s-s02) + 2*b_Y2*(s-s02) + c_Y2) ;

x_path = lambda*x_path1 + (1 - lambda)*x_path2;
y_path = lambda*y_path1 + (1 - lambda)*y_path2;
dx_path = lambda*dx_path1 + (1 - lambda)*dx_path2;
dy_path = lambda*dy_path1 + (1 - lambda)*dy_path2; 

abs_grad = sqrt(dx_path^2 + dy_path^2);
dx_path_norm = dx_path/abs_grad;
dy_path_norm = dy_path/abs_grad;

error_contour = dy_path_norm * (x - x_path) - dx_path_norm * (y - y_path);
error_lag = -dx_path_norm * (x - x_path) - dy_path_norm * (y - y_path);
%vref = lambda*vref1 + (1 - lambda)*vref2;
%% Total cost
cost = Wx*error_contour*error_contour + Wy*error_lag*error_lag +Wv*(v-vref)*(v-vref) +Wa*a*a+ Wdelta*(delta)*(delta)+ws*sv;

end
