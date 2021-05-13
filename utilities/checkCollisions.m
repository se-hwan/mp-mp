function [newPath, collision] = checkCollisions(motionPlan, obstacles)
    collision = false;
    path = motionPlan.X(4:6,:);
    for i=1:length(path(1,:))
        for j=1:length(obstacles(1,:))
            epsilon = obstacles(4,j)/4; delta = epsilon + 0.2;
            if (norm(obstacles(1:3,j) - path(:,i)) < (obstacles(4,j)+0.2))
                obstacle_direction = (path(:,i)-obstacles(1:3,j))/norm(obstacles(1:3,j) - path(:,i));
                newPt = obstacles(1:3,j) + (delta + obstacles(4,j))*obstacle_direction;
                path(:,i) = newPt;
                collision = true;
            end
        end
    end
    newPath = path;
end