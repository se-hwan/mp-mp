function [] = animation(plotData, saveAni)
    
    %% unpack plotData object
    target = plotData.target;
    state = plotData.state;
    input = plotData.input;
    t_sim = plotData.t_sim;
    replannedPaths = plotData.replannedPaths;
    replan_idx = plotData.replan_idx;
    obstacles = plotData.obstacles;
    
    n_plotPoints = numel(state(4,:));
    dt_sim = gradient(t_sim);
    
    %% animation parameters
    figure(2); grid on; hold on;
    a = .5; b = .25; c = 0.25; lod = 30;
    replan = 1; plan_idx = 1; color_idx = 1;
    C = {'k','b','r','g','c','m','y',...
        [0 0.4470 0.7410],[0.8500 0.3250 0.0980],[0.9290 0.6940 0.1250],...
        [0.4940 0.1840 0.5560],[0.4660 0.6740 0.1880],[0.3010 0.7450 0.9330],...
        [1 0.4 0.6]};
    scr_size = get(0,'ScreenSize') ;
    set(gcf, 'Position',  [scr_size(3)/2-960, scr_size(4)/2-540, 1920, 1080])
    xlabel('X (m)')
    ylabel('Y (m)')
    zlabel('Z (m)')
    
    %% Initialize video
    if (saveAni)
        anim = VideoWriter('motionPlan', 'MPEG-4'); %open video file
        anim.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
        open(anim)
    end
    
    %% animation
    roll = state(1,:); pitch = state(2,:); yaw = state(3,:);
    xPosn = state(4,:); yPosn = state(5,:); zPosn = state(6,:);
    force = input(1:3,:); tau = input(4:6,:);
    axis equal;
    xlim([-1.5, target(4) + 1.5]);
    ylim([-1.5, target(5) + 1.5]);
    zlim([-1.5, target(6) + 1.5]);
    
    view([15 30])
    plot3(target(4), target(5), target(6), 'bo') % target position
    
    % plot target
    [X,Y,Z] = cylinder(1, lod);
    R_adjust = rpyToR([0; pi/2; 0]);
    R_target = rpyToR([-target(1); target(2); target(3)])*R_adjust;
    for i = 1:length(X(1,:))
        for j = 1:length(X(:,1))
            s_rot = R_target*[X(j,i); Y(j,i); Z(j,i)];
            X(j,i) = s_rot(1); Y(j,i) = s_rot(2); Z(j,i) = s_rot(3);
        end
    end
    target_plot = surf(X+target(4), Y+target(5), Z+target(6));
    set(target_plot, 'EdgeAlpha', .5, 'FaceColor', 'green', 'FaceAlpha', 0.5)
    
    % plot initial position of ship
    [X,Y,Z] = ellipsoid(0, 0, 0, a, b, c, lod);
    R = rpyToR([roll(1); pitch(1); yaw(1)]);
    for i = 1:length(X(1,:))
        for j = 1:length(X(:,1))
            s_rot = R*[X(j,i); Y(j,i); Z(j,i)];
            X(j,i) = s_rot(1); Y(j,i) = s_rot(2); Z(j,i) = s_rot(3);
        end
    end
    ship = surf(X+xPosn(1), Y+yPosn(1), Z+zPosn(1));
    set(ship, 'EdgeAlpha', 0.05)
    
    % plot obstacles
    if (~isempty(obstacles))
        for i = 1:numel(obstacles(1,:))
            [X,Y,Z] = sphere;
            obs = surf(obstacles(4,i)*X + obstacles(1,i), ...
                 obstacles(4,i)*Y + obstacles(2,i), ...
                 obstacles(4,i)*Z + obstacles(3,i));
            set(obs,'FaceColor','red','FaceAlpha',0.5,'EdgeAlpha',0.01)
        end
    end
    
    
    % plot obstacle avoidance paths
    obstaclesPath = false;
    if (numel(replan_idx) < 2 && (numel(replannedPaths) > 1))
        j = 1;
        obstaclesPath = true;
        for i=1:numel(replannedPaths)
            xPath = replannedPaths{i}(1, :);
            yPath = replannedPaths{i}(2, :);
            zPath = replannedPaths{i}(3, :);
            if (j > numel(C))
                j = 1;
            end
            plannedPath = plot3(xPath(1:end), yPath(1:end), zPath(1:end),...
                            'color',C{j},'linestyle','--');
            pause(0.25);
            delete(plannedPath);
            j = j+1;
        end
    end
    
    pause(.5);
    delete(ship); 
    
    for k = 1:n_plotPoints-1
        title(sprintf('Time: %.2f s', vpa(t_sim(k),2))); % time text
        
        % plot ellipse
        [X,Y,Z] = ellipsoid(0, 0, 0, a, b, c, lod);
        R = rpyToR([roll(k); pitch(k); yaw(k)]);
        for i = 1:length(X(1,:))
            for j = 1:length(X(:,1))
                s_rot = R*[X(j,i); Y(j,i); Z(j,i)];
                X(j,i) = s_rot(1); Y(j,i) = s_rot(2); Z(j,i) = s_rot(3);
            end
        end
        ship = surf(X+xPosn(k), Y+yPosn(k), Z+zPosn(k));
        set(ship, 'EdgeAlpha', 0.05)
        
        % plot force and torque vectors
        r_scale = .01;
        f_k = -R*force(:,k); tau_k = -R*tau(:,k);
        f_mag = norm(f_k); tau_mag = norm(tau_k);
        f_dir = f_k/f_mag; tau_dir = tau_k/tau_mag;
        
        p = [xPosn(k) yPosn(k) zPosn(k)];
        f_vect = quiver3(p(1),p(2),p(3), ...
                         p(1)+f_mag*f_dir(1), ...
                         p(2)+f_mag*f_dir(2), ...
                         p(3)+f_mag*f_dir(3),f_mag*r_scale,'r-');
        tau_vect = quiver3(p(1),p(2),p(3), ...
                           p(1)+tau_mag*tau_dir(1), ...
                           p(2)+tau_mag*tau_dir(2), ...
                           p(3)+tau_mag*tau_dir(3),tau_mag*r_scale,'b-');
        
        % plot initial path
        if (k == replan_idx(replan))
            xPath = replannedPaths{replan}(1, :);
            yPath = replannedPaths{replan}(2, :);
            zPath = replannedPaths{replan}(3, :);
            if (replan < length(replan_idx))
                replan = replan + 1;
                color_idx = color_idx + 1;
            end
            plan_idx = 1;
        end
        if (color_idx == length(C))
            color_idx = 1;
        end
        plannedPath = plot3(xPath(plan_idx:end), yPath(plan_idx:end), zPath(plan_idx:end),...
                            'color',C{color_idx},'linestyle','--');
        
        if (obstaclesPath)
            delete(plannedPath);
            xPath = replannedPaths{end}(1, :);
            yPath = replannedPaths{end}(2, :);
            zPath = replannedPaths{end}(3, :);
            plannedPath = plot3(xPath(k:end), yPath(k:end), zPath(k:end),...
                            'color',C{1},'linestyle','--');
        end
                        
        pause(dt_sim/4);
        
        if (saveAni)
            frame = getframe(gcf); %get frame
            writeVideo(anim, frame);
        end
            
        delete(f_vect); delete(tau_vect);
        delete(ship); delete(plannedPath);
        
        plan_idx = plan_idx + 1;
    end
    % plot final state
    [X,Y,Z] = ellipsoid(0, 0, 0, ...
                        a, b, c, lod);
    R = rpyToR([roll(end); pitch(end); yaw(end)]);
    for i = 1:length(X(1,:))
        for j = 1:length(X(:,1))
            s_rot = R*[X(j,i); Y(j,i); Z(j,i)];
            X(j,i) = s_rot(1); Y(j,i) = s_rot(2); Z(j,i) = s_rot(3);
        end
    end
    ship = surf(X+xPosn(end), Y+yPosn(end), Z+zPosn(end));
    set(ship, 'EdgeAlpha', 0.05)

    if (saveAni)
        frame = getframe(gcf); %get frame
        writeVideo(anim, frame);
        close(anim)
    end
    
    
    hold off;
end