function [cost] =  objective_scenario( z, p,i)
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
    %%Parameters
    r_disc = p(27); disc_pos_0 = p(28);
    obst1_x = p(29); obst1_y = p(30); obst1_theta = p(31); obst1_major = p(32); obst1_minor= p(33);
    obst2_x = p(34); obst2_y = p(35); obst2_theta = p(36); obst2_major = p(37); obst2_minor= p(38);
    obst3_x = p(39); obst3_y = p(40); obst3_theta = p(41); obst3_major = p(42); obst3_minor= p(43);
    obst4_x = p(44); obst4_y = p(45); obst4_theta = p(46); obst4_major = p(47); obst4_minor= p(48);
    obst5_x = p(49); obst5_y = p(50); obst5_theta = p(51); obst5_major = p(52); obst5_minor= p(53);
    obst6_x = p(54); obst6_y = p(55); obst6_theta = p(56); obst6_major = p(57); obst6_minor= p(58);
    
    %% Collision Avoidance Constraints
	R_car= [cos(theta), -sin(theta); sin(theta), cos(theta)];
	CoG = [x;y];

	shift_0 = [disc_pos_0; 0];

    % Car disc positions
	position_disc_0 = CoG+R_car*shift_0;
    
    %% Obstacles
    % Obstacle 1
	CoG_obst1 = [obst1_x;obst1_y];
	deltaPos_disc_0_obstacle_1 =  position_disc_0 - CoG_obst1;

    % Obstacle 2
	CoG_obst2 = [obst2_x;obst2_y];
	deltaPos_disc_0_obstacle_2 =  position_disc_0 - CoG_obst2;
    
    % Obstacle 3
	CoG_obst3 = [obst3_x;obst3_y];
	deltaPos_disc_0_obstacle_3 =  position_disc_0 - CoG_obst3;

    % Obstacle 4
	CoG_obst4 = [obst4_x;obst4_y];
	deltaPos_disc_0_obstacle_4 =  position_disc_0 - CoG_obst4;
    
    % Obstacle 5
	CoG_obst5 = [obst5_x;obst5_y];
	deltaPos_disc_0_obstacle_5 =  position_disc_0 - CoG_obst5;

    % Obstacle 6
	CoG_obst6 = [obst6_x;obst6_y];
	deltaPos_disc_0_obstacle_6 =  position_disc_0 - CoG_obst6;
    
    %% Relative Rotation Matrix
	ab_1 = [1/((obst1_major + r_disc)*(obst1_major + r_disc)),0;0,1/((obst1_minor + r_disc)*(obst1_minor + r_disc))];
	ab_2 = [1/((obst2_major + r_disc)*(obst2_major + r_disc)),0;0,1/((obst2_minor + r_disc)*(obst2_minor + r_disc))];
    ab_3 = [1/((obst3_major + r_disc)*(obst3_major + r_disc)),0;0,1/((obst3_minor + r_disc)*(obst3_minor + r_disc))];
	ab_4 = [1/((obst4_major + r_disc)*(obst4_major + r_disc)),0;0,1/((obst4_minor + r_disc)*(obst4_minor + r_disc))];
    ab_5 = [1/((obst5_major + r_disc)*(obst5_major + r_disc)),0;0,1/((obst5_minor + r_disc)*(obst5_minor + r_disc))];
	ab_6 = [1/((obst6_major + r_disc)*(obst6_major + r_disc)),0;0,1/((obst6_minor + r_disc)*(obst6_minor + r_disc))];

	R_obst_1 = [cos(obst1_theta), -sin(obst1_theta);sin(obst1_theta),cos(obst1_theta)];
	R_obst_2 = [cos(obst2_theta), -sin(obst2_theta);sin(obst2_theta),cos(obst2_theta)];
    R_obst_3 = [cos(obst3_theta), -sin(obst3_theta);sin(obst3_theta),cos(obst3_theta)];
	R_obst_4 = [cos(obst4_theta), -sin(obst4_theta);sin(obst4_theta),cos(obst4_theta)];
    R_obst_5 = [cos(obst5_theta), -sin(obst5_theta);sin(obst5_theta),cos(obst5_theta)];
	R_obst_6 = [cos(obst6_theta), -sin(obst6_theta);sin(obst6_theta),cos(obst6_theta)];

    %% Constraints
    c_disc_0_obst_1 = deltaPos_disc_0_obstacle_1' * R_obst_1' * ab_1 * R_obst_1 * deltaPos_disc_0_obstacle_1;
    c_disc_0_obst_2 = deltaPos_disc_0_obstacle_2' * R_obst_2' * ab_2 * R_obst_2 * deltaPos_disc_0_obstacle_2;
    c_disc_0_obst_3 = deltaPos_disc_0_obstacle_3' * R_obst_3' * ab_3 * R_obst_3 * deltaPos_disc_0_obstacle_3;
    c_disc_0_obst_4 = deltaPos_disc_0_obstacle_4' * R_obst_4' * ab_4 * R_obst_4 * deltaPos_disc_0_obstacle_4;
    c_disc_0_obst_5 = deltaPos_disc_0_obstacle_5' * R_obst_5' * ab_5 * R_obst_5 * deltaPos_disc_0_obstacle_5;
    c_disc_0_obst_6 = deltaPos_disc_0_obstacle_6' * R_obst_6' * ab_6 * R_obst_6 * deltaPos_disc_0_obstacle_6;
% Bivariate Gaussian
%field1 = 1/(2*pi)*det(inv(sigma))*exp(-0.5*(deltaPos_disc_0_obstacle_1)'*inv(sigma)*deltaPos_disc_0_obstacle_1);
%field2 = 1/(2*pi)*det(inv(sigma))*exp(-0.5*(deltaPos_disc_0_obstacle_2)'*inv(sigma)*deltaPos_disc_0_obstacle_2);

%% Total cost
%cost = Wcontour*error_contour*error_contour + Wlag*error_lag*error_lag +Wv*(v-vref)*(v-vref) +Ww*w*w+ws*sv*sv+Wrepulsive*(field1 + field2);

%% Total cost (0.9^i)*(
cost = (1.0^i)*(Wcontour*error_contour*error_contour + Wlag*error_lag*error_lag) +Wv*(v-vref)*(v-vref) +Ww*w*w+ws*sv*sv+Wrepulsive*(1/((1-c_disc_0_obst_1)*(1-c_disc_0_obst_1)+.001)+1/((1-c_disc_0_obst_2)*(1-c_disc_0_obst_2)+.001)+1/((1-c_disc_0_obst_3)*(1-c_disc_0_obst_3)+.001)+1/((1-c_disc_0_obst_4)*(1-c_disc_0_obst_4)+.001)+1/((1-c_disc_0_obst_5)*(1-c_disc_0_obst_5)+.001)+1/((1-c_disc_0_obst_6)*(1-c_disc_0_obst_6)+.001));

end
