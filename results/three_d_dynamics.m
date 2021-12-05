function state_dot = three_d_dynamics(t, state, g, max_eta, max_rho, max_T)
    x = state(1);y = state(2);z = state(3);x_dot = state(4);y_dot = state(5)...
        ;z_dot = state(6); eta = state(7); rho = state(8); T = state(9);
    
    %% Pose
    x_dotdot = -T*sin(eta)*cos(rho);
    y_dotdot = -T*sin(eta)*sin(rho);
    z_dotdot = T*cos(eta) - g;
    
    %% Controls
    eta_dot = 0;
    rho_dot = 0;
    T_dot = 0;
    
    %% State derivative
    state_dot = [x_dot; y_dot; z_dot; x_dotdot; y_dotdot; z_dotdot; eta_dot; rho_dot; T_dot];
end