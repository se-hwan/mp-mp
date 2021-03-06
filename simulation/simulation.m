function output = simulation(model, state_init, target, obstacles, options)
    %% state vectors
    state = zeros(model.N_states);
    input = zeros(model.N_inputs);
    state(:,1) = state_init;

    %% initial motion planning
    motionPlan_init = motionPlanner(model, state_init, target, []);
    motionPlan = motionPlan_init;
    replannedPaths{1} = [motionPlan.X(4,:); motionPlan.X(5,:); motionPlan.X(6,:)];

    %% simulation
    i = 1; j = 1; replan = 1;
    tol = .25; % final state convergence
    threshold = 1; % state drift threshold
    t_sim = zeros(1,1);
    dt_sim = motionPlan.time(2) - motionPlan.time(1);
    replan_idx = ones(1,1);

    switch options.simType
        case 'deterministic' % deterministic system
            while(norm(state(:,i) - target) > tol)    
                state(:,i+1) = state(:,i) + dt_sim*(model.f(state(:,i),motionPlan.U(:,j)));
                input(:,i) = motionPlan.U(:,j);
                t_sim(i+1) = t_sim(i) + dt_sim;
                if ((j == 50) || (norm(state(:,i+1) - motionPlan.X(:,j+1))) > threshold) % this will never trigger without noise in the system
                    disp('Replan')
                    motionPlan = motionPlanner(model, state(:,i+1), target, motionPlan); % replan only if current state deviates significantly from planned state
                    replan_idx(replan + 1) = i+1;
                    replannedPaths{replan+1} = [motionPlan.X(4,:); motionPlan.X(5,:); motionPlan.X(6,:)];
                    replan = replan + 1;
                    dt_sim = 0.05;
                    j = 1;
                end
                i = i+1; j = j+1;
            end
        case 'noisy' % Gaussian noise in all states
            while(norm(state(:,i) - target) > tol && replan < 25)
                noiseScale = .1;
                state(:,i+1) = state(:,i) + dt_sim*(model.f(state(:,i) + [noiseScale*randn(12,1)],motionPlan.U(:,j)));
                input(:,i) = motionPlan.U(:,j);
                t_sim(i+1) = t_sim(i) + dt_sim;
                disp('Replan')
                motionPlan = motionPlanner(model, state(:,i+1), target, motionPlan); % replan only if current state deviates significantly from planned state
                replan_idx(replan + 1) = i+1;
                replannedPaths{replan+1} = [motionPlan.X(4,:); motionPlan.X(5,:); motionPlan.X(6,:)];
                %plot3(motionPlan.X(4,:), motionPlan.X(5,:), motionPlan.X(6,:))
                replan = replan + 1;
                j = 1;
                i = i+1; j = j+1;
            end
        case 'obstacles' % obstacles to avoid in path
            figure; grid on; hold on;
            tic
            dt_sim = motionPlan.time(2) - motionPlan.time(1);
            while(1)
                [path, collision] = checkCollisions(motionPlan, obstacles);
                if (collision)
                    motionPlan = motionPlanner_obstacles(model, state(:,1), target, motionPlan, path, replan);
                    replannedPaths{replan+1} = [motionPlan.X(4,:); motionPlan.X(5,:); motionPlan.X(6,:)];
                    plot3(motionPlan.X(4,:), motionPlan.X(5,:), motionPlan.X(6,:))
                    replan = replan + 1;
                else
                    break;
                end
            end
            toc
            while(norm(state(:,i) - target) > tol && i < 51)   
                state(:,i+1) = state(:,i) + dt_sim*(model.f(state(:,i), motionPlan.U(:,j)));
                input(:,i) = motionPlan.U(:,j);
                t_sim(i+1) = t_sim(i) + dt_sim;
                i = i+1; j = j+1;
            end

    end

    output.state = state;
    output.input = input;
    output.t_sim = t_sim;
    output.replannedPaths = replannedPaths;
    output.replan_idx = replan_idx;
    
end