function ineq_constr =  inequality_constr_scenario( z, p, i)
    
    % states and inputs for ego vehicle
    x_R = z(4: 9);
    u_R = z(1: 3);
    x = x_R(1);
    y = x_R(2);
    theta = x_R(3);
    slack = u_R(3);

    %% Parameters
    r_disc = p(28); disc_pos_0 = p(29);disc_pos_1 = p(30); disc_pos_2 = p(31);
    obst1_x = p(32); obst1_y = p(33); obst1_theta = p(34); obst1_major = p(35); obst1_minor= p(36);
    obst2_x = p(37); obst2_y = p(38); obst2_theta = p(39); obst2_major = p(40); obst2_minor= p(41);
    
    %% Collision Avoidance Constraints
	R_car= [cos(theta), -sin(theta); sin(theta), cos(theta)];
	CoG = [x;y];

	shift_0 = [disc_pos_0; 0];
	shift_1 = [disc_pos_1; 0];
    shift_2 = [disc_pos_2; 0];

    % Car disc positions
	position_disc_0 = CoG+R_car*shift_0;
	position_disc_1 = CoG+R_car*shift_1;
    position_disc_2 = CoG+R_car*shift_2;
    
    % Obstacle 1
	CoG_obst1 = [obst1_x;obst1_y];
	deltaPos_disc_0_obstacle_1 =  position_disc_0 - CoG_obst1;
	deltaPos_disc_1_obstacle_1 =  position_disc_1 - CoG_obst1;
    deltaPos_disc_2_obstacle_1 =  position_disc_2 - CoG_obst1;

    % Obstacle 2
	CoG_obst2 = [obst2_x;obst2_y];
	deltaPos_disc_0_obstacle_2 =  position_disc_0 - CoG_obst2;
	deltaPos_disc_1_obstacle_2 =  position_disc_1 - CoG_obst2;
    deltaPos_disc_2_obstacle_2 =  position_disc_2 - CoG_obst2;
    
    % Relative Rotation Matrix
	ab_1 = [1/((obst1_major + r_disc)*(obst1_major + r_disc)),0;0,1/((obst1_minor + r_disc)*(obst1_minor + r_disc))];
	ab_2 = [1/((obst2_major + r_disc)*(obst2_major + r_disc)),0;0,1/((obst2_minor + r_disc)*(obst2_minor + r_disc))];

	R_obst_1 = [cos(obst1_theta), -sin(obst1_theta);sin(obst1_theta),cos(obst1_theta)];
	R_obst_2 = [cos(obst2_theta), -sin(obst2_theta);sin(obst2_theta),cos(obst2_theta)];

    % Constraints
    c_disc_0_obst_1 = deltaPos_disc_0_obstacle_1' * R_obst_1' * ab_1 * R_obst_1 * deltaPos_disc_0_obstacle_1;
	c_disc_1_obst_1 = deltaPos_disc_1_obstacle_1' * R_obst_1' * ab_1 * R_obst_1 * deltaPos_disc_1_obstacle_1;
	c_disc_2_obst_1 = deltaPos_disc_2_obstacle_1' * R_obst_1' * ab_1 * R_obst_1 * deltaPos_disc_2_obstacle_1;

    c_disc_0_obst_2 = deltaPos_disc_0_obstacle_2' * R_obst_2' * ab_2 * R_obst_2 * deltaPos_disc_0_obstacle_2;
	c_disc_1_obst_2 = deltaPos_disc_1_obstacle_2' * R_obst_2' * ab_2 * R_obst_2 * deltaPos_disc_1_obstacle_2;
	c_disc_2_obst_2 = deltaPos_disc_2_obstacle_2' * R_obst_2' * ab_2 * R_obst_2 * deltaPos_disc_2_obstacle_2;

    ineq_constr = [c_disc_0_obst_1 + slack;
        c_disc_1_obst_1 + slack;
        c_disc_2_obst_1 + slack;
        c_disc_0_obst_2 + slack;
        c_disc_1_obst_2 + slack;
        c_disc_2_obst_2 + slack];
                  
end
    