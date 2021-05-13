function path = motionPlanner(model, state, target_state, prevMotionPlan)
    import casadi.*
    
    %% SET OPTIMIZATION PARAMETERS
    param.n_pts_planning = 50;

    %% OPTIMIZATION
    opti = casadi.Opti(); % Optimization stack
    soln_init = optimization(opti, model, target_state, param, state, prevMotionPlan);
    path = soln_init;
end

function soln = optimization(opti, model, target_state, param, state, prevSol)
	%% LOAD PARAMETERS
    n_states = model.N_states;
    n_inputs = model.N_inputs;
    maxThrust = model.MaxThrust;
    maxTorque = model.MaxTorque;
    
    n_pts = param.n_pts_planning;
    
    firstRun = false;
    if isempty(prevSol)
        firstRun = true;
    end
    
    X_init = state;
    X_term = target_state;

    %% DECISION VARIABLES
    X = opti.variable(n_states,n_pts+1); % state trajectories
    
    U = opti.variable(n_inputs,n_pts);   % control trajectories (throttle)
    U_max = opti.variable(n_inputs,n_pts);   % absolute value slack variable
    
    T = opti.variable();      % final time

    %% DYNAMIC CONSTRAINTS AND COST
    f = @(x,u) model.f(x,u); % dx/dt = f(x,u)
    dt = T/n_pts; % length of a control interval
    cost = 0;

    if (firstRun)
        for k=1:n_pts % loop over control intervals
           x_next = X(:,k) + dt * f(X(:,k),U(:,k));
           opti.subject_to(X(:,k+1)==x_next);
           cost = cost + 10*dt; % min-time min-fuel formulation
           for j=1:n_inputs % enforces L-1 norm minimization
               opti.subject_to(U(j,k) <= U_max(j,k))
               opti.subject_to(-U(j,k) <= U_max(j,k))
               cost = cost + .1*U_max(j,k);
           end
           %cost = cost + 100*dt + .01*U(:,k)'*U(:,k); % min-time min-power formulation
        end
    else
        dt = prevSol.time(2) - prevSol.time(1);
        for k=1:n_pts
            x_next = X(:,k) + dt * f(X(:,k),U(:,k));
            opti.subject_to(X(:,k+1) == x_next);
            for j=1:n_inputs % enforces L-1 norm minimization
               opti.subject_to(U(j,k) <= U_max(j,k))
               opti.subject_to(-U(j,k) <= U_max(j,k))
               cost = cost + .1*U_max(j,k);
            end
        end
    end

    opti.minimize(cost); % objective
    
    %% INPUT CONSTRAINTS
    for i=1:n_pts
        opti.subject_to(U(1,i) <= maxThrust)    % max thrust constraint
        opti.subject_to(-U(1,i) <= maxThrust)
        opti.subject_to(U(2,i) <= 0)
        opti.subject_to(-U(2,i) <= 0)
        opti.subject_to(U(3,i) <= 0)
        opti.subject_to(-U(3,i) <= 0)
        opti.subject_to(U(4,i) <= maxTorque)    % max torque constraint
        opti.subject_to(-U(4,i) <= maxTorque)
        opti.subject_to(U(5,i) <= maxTorque)
        opti.subject_to(-U(5,i) <= maxTorque)
        opti.subject_to(U(6,i) <= maxTorque)   
        opti.subject_to(-U(6,i) <= maxTorque)
    end

    %% STATE CONSTRAINTS
    opti.subject_to(X(:,1) == X_init);          % initial condition
    opti.subject_to(X(:,n_pts+1) <= X_term(:));    % terminal condition
    opti.subject_to(X(:,n_pts+1) >= X_term(:));    % terminal condition
    opti.subject_to(T>=0);                      % positive time

    %% INTIIAL GUESSES
    if (~firstRun)
        %opti.set_initial(X(:,:), prevSol.X); 
        %opti.set_initial(U(:,:), prevSol.U);
    end

    %% SOLVE OPTIMIZATION
    p_opts = struct('expand',true); % this speeds up ~x10
    p_opts.print_time = 0; 
    s_opts.print_level = 0; s_opts.file_print_level = 0;
    s_opts.print_timing_statistics ='no';
    opti.solver('ipopt', p_opts, s_opts);
    sol = opti.solve();   % actual solve
    
    %% UNPACK SOLUTION
    if (firstRun)
        soln.time = 0:sol.value(T)/n_pts:sol.value(T);
    else
        soln.time = 0:dt:prevSol.time(end);
    end
    soln.X = sol.value(X);
    soln.U = sol.value(U);
end