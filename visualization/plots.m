function [] = plots(plotData, model)
    
    %% unpack plotData object
    target = plotData.target;
    state = plotData.state;
    input = plotData.input;
    t_sim = plotData.t_sim;
    replannedPaths = plotData.replannedPaths;
    replan_idx = plotData.replan_idx;
    obstacles = plotData.obstacles;
    
    %% state plots
    figure; hold on;                               % roll, pitch, yaw
    plot(t_sim, rad2deg(state(1,:)),'r-')
    plot(t_sim, rad2deg(state(2,:)),'g-')
    plot(t_sim, rad2deg(state(3,:)),'b-')
    plot(t_sim,rad2deg(target(1))*ones(numel(t_sim),1), 'r--')
    plot(t_sim,rad2deg(target(2))*ones(numel(t_sim),1), 'g--')
    plot(t_sim,rad2deg(target(3))*ones(numel(t_sim),1), 'b--')
    xlabel('Time (s)'); ylabel('Orientation (deg)')
    legend('Roll', 'Pitch', 'Yaw','Roll_d','Pitch_d','Yaw_d')
    hold off
    
    figure; hold on;                               % position
    plot(t_sim, state(4,:),'r-')
    plot(t_sim, state(5,:),'g-')
    plot(t_sim, state(6,:),'b-')
    plot(t_sim,target(4)*ones(numel(t_sim),1), 'r--')
    plot(t_sim,target(5)*ones(numel(t_sim),1), 'g--')
    plot(t_sim,target(6)*ones(numel(t_sim),1), 'b--')
    legend('x','y','z','x_d','y_d','z_d')
    xlabel('Time (s)'); ylabel('Position (m)')
    hold off
    
    figure; hold on;                               % angular velocity
    plot(t_sim, state(7,:),'r-')
    plot(t_sim, state(8,:),'g-')
    plot(t_sim, state(9,:),'b-')
    plot(t_sim,target(7)*ones(numel(t_sim),1), 'r--')
    plot(t_sim,target(8)*ones(numel(t_sim),1), 'g--')
    plot(t_sim,target(9)*ones(numel(t_sim),1), 'b--')
    xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)')
    legend('\omega_x', '\omega_y', '\omega_z','\omega_{x,d}','\omega_{y,d}','\omega_{z,d}')
    hold off
    
    figure; hold on;                               % linear velocity
    plot(t_sim, state(10,:),'r-')
    plot(t_sim, state(11,:),'g-')
    plot(t_sim, state(12,:),'b-')
    plot(t_sim,target(10)*ones(numel(t_sim),1), 'r--')
    plot(t_sim,target(11)*ones(numel(t_sim),1), 'g--')
    plot(t_sim,target(12)*ones(numel(t_sim),1), 'b--')
    xlabel('Time (s)'); ylabel('Linear Velocity (m/s)')
    legend('v_x', 'v_y', 'v_z','v_{x,d}','v_{y,d}','v_{z,d}')
    hold off
    
    %% input plots
    figure; hold on;
    plot(t_sim(1:end-1), input(1,:),'r-')
%     plot(t_sim(1:end-1), input(2,:),'g-')
%     plot(t_sim(1:end-1), input(3,:),'b-')    
    plot(t_sim(1:end-1), model.MaxThrust*ones(length(t_sim(1:end-1)),1),'k--')
    plot(t_sim(1:end-1), -model.MaxThrust*ones(length(t_sim(1:end-1)),1),'k--')
    legend('F_x','F_{max}')
    xlabel('Time (s)')
    ylabel('Force (N)')
    hold off;

    figure; hold on;
    plot(t_sim(1:end-1), input(4,:),'r-')
    plot(t_sim(1:end-1), input(5,:),'g-')
    plot(t_sim(1:end-1), input(6,:),'b-')    
    plot(t_sim(1:end-1), model.MaxTorque*ones(length(t_sim(1:end-1)),1),'k--')
    plot(t_sim(1:end-1), -model.MaxTorque*ones(length(t_sim(1:end-1)),1),'k--')
    legend('\tau_x','\tau_y','\tau_z','\tau_{max}')
    xlabel('Time (s)')
    ylabel('Torque (N)')
    hold off;
    
    %% replanning
    figure; hold on;
    xPath = replannedPaths{1}(1,:); 
    yPath = replannedPaths{1}(2,:); 
    zPath = replannedPaths{1}(3,:);
    h(1) = plot3(xPath,yPath,zPath, 'bo-');
    for i = 2:numel(replannedPaths)-1
        xPath = replannedPaths{i}(1,:);
        yPath = replannedPaths{i}(2,:);
        zPath = replannedPaths{i}(3,:);
        plot3(xPath,yPath,zPath)
    end
    xPath = replannedPaths{end}(1,:); 
    yPath = replannedPaths{end}(2,:); 
    zPath = replannedPaths{end}(3,:);
    h(2) = plot3(xPath,yPath,zPath, 'go-');
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)')
    legend(h, 'Initial path', 'Final path','AutoUpdate','off')
        
    % plot obstacles
    if (~isempty(obstacles))
        grid on;
        for i = 1:numel(obstacles(1,:))
            [X,Y,Z] = sphere;
            obs = surf(obstacles(4,i)*X + obstacles(1,i), ...
                 obstacles(4,i)*Y + obstacles(2,i), ...
                 obstacles(4,i)*Z + obstacles(3,i));
            set(obs,'FaceColor','red','FaceAlpha',0.5,'EdgeAlpha',0.01)
        end
        axis equal;
        xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)')
        hold off;
    end

    hold off;
end