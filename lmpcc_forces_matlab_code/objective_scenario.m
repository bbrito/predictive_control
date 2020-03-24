function [cost] =  objective_scenario( z, p)
%% Cost for ego-vehicle    
% states and inputs for ego vehicle
%            inputs               |               states
%                v   w  sv     x      y       theta      dummy
x_R = z(4: 7);
u_R = z(1: 2);
v = u_R(1);
w = u_R(2);
sv = z(3);
x = x_R(1);
y = x_R(2);
theta = x_R(3);
s = x_R(4);

%% Spline parameters
s01 = p(17);
s02 = p(18);
d = p(19);

a_X1 = p(1); b_X1 = p(2); c_X1 = p(3); d_X1 = p(4);
a_Y1 = p(5); b_Y1 = p(6); c_Y1 = p(7); d_Y1 = p(8);
a_X2 = p(9); b_X2 = p(10); c_X2 = p(11); d_X2 = p(12);
a_Y2 = p(13); b_Y2 = p(14); c_Y2 = p(15); d_Y2 = p(16);

% Weights
Wcontour = p(20);
Wlag = p(21);
Ww = p(22);
vref = p(23);
ws = p(24);
Wrepulsive = p(25);
Wv = p(26);

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

%% Obstacles Soft-Constraints
r_disc = p(27); disc_pos_0 = p(28);
obst1_x = p(29); obst1_y = p(30); obst1_theta = p(31); obst1_major = p(32); obst1_minor= p(33);
obst2_x = p(34); obst2_y = p(35); obst2_theta = p(36); obst2_major = p(37); obst2_minor= p(38);
    
R_car= [cos(theta), -sin(theta); sin(theta), cos(theta)];
CoG = [x;y];

shift_0 = [disc_pos_0; 0];

% Car disc positions
position_disc_0 = CoG+R_car*shift_0;

% Obstacle 1
CoG_obst1 = [obst1_x;obst1_y];
deltaPos_disc_0_obstacle_1 =  position_disc_0 - CoG_obst1;

% Obstacle 2
CoG_obst2 = [obst2_x;obst2_y];
deltaPos_disc_0_obstacle_2 =  position_disc_0 - CoG_obst2;
    
% Relative Rotation Matrix
ab_1 = [1/((obst1_major + r_disc)*(obst1_major + r_disc)),0;0,1/((obst1_minor + r_disc)*(obst1_minor + r_disc))];
ab_2 = [1/((obst2_major + r_disc)*(obst2_major + r_disc)),0;0,1/((obst2_minor + r_disc)*(obst2_minor + r_disc))];

R_obst_1 = [cos(obst1_theta), -sin(obst1_theta);sin(obst1_theta),cos(obst1_theta)];
R_obst_2 = [cos(obst2_theta), -sin(obst2_theta);sin(obst2_theta),cos(obst2_theta)];

% Constraints
c_disc_0_obst_1 = deltaPos_disc_0_obstacle_1' * R_obst_1' * ab_1 * R_obst_1 * deltaPos_disc_0_obstacle_1;

c_disc_0_obst_2 = deltaPos_disc_0_obstacle_2' * R_obst_2' * ab_2 * R_obst_2 * deltaPos_disc_0_obstacle_2;

%% Total cost
cost = Wcontour*error_contour*error_contour + Wlag*error_lag*error_lag +Wv*(v-vref)*(v-vref) +Ww*w*w+sv*sv;%+Wrepulsive*(1/((1-c_disc_0_obst_1)*(1-c_disc_0_obst_1)+.001))+Wrepulsive*(1/((1-c_disc_1_obst_1)*(1-c_disc_1_obst_1)+.001))+Wrepulsive*(1/((1-c_disc_2_obst_1)*(1-c_disc_2_obst_1)+.001))+Wrepulsive*(1/((1-c_disc_0_obst_2)*(1-c_disc_0_obst_2)+.001))+Wrepulsive*(1/((1-c_disc_1_obst_2)*(1-c_disc_1_obst_2)+.001))+Wrepulsive*(1/((1-c_disc_2_obst_2)*(1-c_disc_2_obst_2)+.001));

end
