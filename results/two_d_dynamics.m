function state_dot = two_d_dynamics(t, state, g, max_eta, max_rho, max_T)
    x = state(1);y = state(2);x_dot = state(3);y_dot = state(4);eta = state(5); T = state(6);
    
    %% Pose
    x_dotdot = -T*sin(eta);
    y_dotdot = T*cos(eta) - g;
    
    %% Controls
    eta_dot = 0;
    T_dot = 0;
    
    %% State derivative
    state_dot = [x_dot; y_dot; x_dotdot; y_dotdot; eta_dot; T_dot];
end