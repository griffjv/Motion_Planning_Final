function state_dot = three_d_dynamics(t, state, g, controls, max_eta, max_rho, max_T)
    x = state(1);y = state(2);z = state(3);x_dot = state(4);y_dot = state(5)...
        ;z_dot = state(6); eta = state(7); rho = state(8); T = state(9);
    
    %% Pose
    x_dotdot = -T*sin(eta)*cos(rho);
    y_dotdot = -T*sin(eta)*sin(rho);
    z_dotdot = T*cos(eta) - g;
    
    %% Controls
   if eta >= max_eta && controls(1)>0
        eta_dot = 0;
   elseif eta <= -max_eta && controls(1)<0
       eta_dot = 0;
   else
       eta_dot = controls(1);
   end

   if rho >= max_rho && controls(2)>0
        rho_dot = 0;
   elseif rho<= -max_eta && controls(2)<0
       rho_dot = 0;
   else
       rho_dot = controls(2);
   end
 
   if T >= max_T && controls(3)>0
        T_dot = 0;
   elseif T<= 0 && controls(3)<0
       T_dot = 0;
   else
       T_dot = controls(3);
   end

    %% State derivative
    state_dot = [x_dot; y_dot; z_dot; x_dotdot; y_dotdot; z_dotdot; eta_dot; rho_dot; T_dot];
end