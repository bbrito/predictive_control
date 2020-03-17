function ineq_constr =  inequality_constr_scenario( z, p, i)
    
    % states and inputs for ego vehicle
    x_R = z(4: 9);
    u_R = z(1: 3);
    x = x_R(1);
    y = x_R(2);
    s = x_R(5);
    slack = u_R(3);
                  
end
    
