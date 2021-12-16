function path = processKinodynamicPath(filename, compare_integrated)
    path = readmatrix(filename);

    %% parameters
    I = 1000; %Assume very large I to ignore rotational rate of craft
    g = 9.81; %Gravitational acceleration Earth, could modify for different bodies
    max_thrust = 2*g; %T/m... tune this parameter to get desired results
    max_eta = pi/4;
    max_rho = pi/4;

    %% initial conditions
    ic3d = path(1,1:9);
    geo_path = ic3d;
    
    %% simulate
    for i = 2:length(path(:,1))
        controls = path(i,10:12);
        duration = path(i,end);

        tspan = [0 duration];
        [t, y] = ode45(@(t,y) three_d_dynamics(t,y,g,controls, max_eta,max_rho,max_thrust), tspan, ic3d);
        
        ic3d = y(end,:);
        geo_path = [geo_path;y];
    end
    geo_path((geo_path(:,3)<0),3) = 0;
    %% plot
    if compare_integrated
        figure
        hold on
        grid on
        plot3(geo_path(:,1),geo_path(:,2),geo_path(:,3), 'b', 'linewidth', 1.5)
        plot3(path(:,1),path(:,2),path(:,3), 'g', 'linewidth', 1.5)
        legend('Integrated Path', 'Geometric Path')
    end
    
end